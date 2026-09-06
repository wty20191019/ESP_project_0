#include "gps.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"

/* ============================================================
 * GPS / GNSS: UART1 + NMEA0183 解析 + PPS 秒脉冲
 *
 * 支持语句: GGA / RMC / GLL / GSA / VTG / ZDA / TXT (天线告警)
 * 多星座前缀(GN/GP/GL/GB 等)通用。
 *
 * 线程模型:
 *   [UART RX] --字节--> parse_task: 组行->校验->解析->更新快照->数据回调
 *   [GPIO9 上升沿] --中断--> ISR(记 esp_timer 时间戳) --> pps_task --> PPS 回调
 * 快照 gps_info_t 由互斥锁保护, gps_get_info() 任意时刻可取一致数据。
 * ============================================================ */

#define TAG "GPS"

#define GPS_RX_BUF_SIZE      (2048)    /* UART RX 环形缓冲 */
#define GPS_LINE_MAX         (160)     /* NMEA 单行最大长度(含校验) */
#define GPS_PARSE_TASK_STACK (4096)
#define GPS_PARSE_TASK_PRIO  (8)
#define GPS_PPS_TASK_STACK   (3072)
#define GPS_PPS_TASK_PRIO    (10)      /* PPS 对齐回调尽量高优先级 */

/* 内部 PPS 事件(中断 -> pps 任务) */
typedef struct {
    int64_t  ts_us;    /* esp_timer 时刻(微秒) */
    uint32_t seq;      /* 上升沿序号 */
} pps_evt_t;

/* ---------- 静态状态 ---------- */
static SemaphoreHandle_t  s_mutex   = NULL;   /* 保护 gps_info_t / 回调指针 */
static QueueHandle_t      s_pps_q   = NULL;   /* PPS 事件队列 */
static TaskHandle_t       s_parse_handle = NULL;
static TaskHandle_t       s_pps_handle   = NULL;

static gps_info_t s_info;                     /* 最新解析快照 */

static gps_pps_callback_t  s_pps_cb     = NULL;
static void               *s_pps_cb_arg = NULL;
static gps_data_callback_t s_data_cb    = NULL;
static void               *s_data_cb_arg = NULL;

static bool s_started = false;

/* PPS 上升沿原子信息(ISR 写, 其它上下文读) */
static portMUX_TYPE s_pps_lock = portMUX_INITIALIZER_UNLOCKED;
static volatile uint32_t s_pps_seq     = 0;
static volatile int64_t  s_pps_last_us = -1;

/* NMEA 行缓冲(仅解析任务访问) */
static char   s_line[GPS_LINE_MAX];
static size_t s_line_len = 0;
static bool   s_line_overflow = false;

/* ============================================================
 * 基础工具
 * ============================================================ */

/* 取第 n 个逗号分隔字段的起始指针(从 0 计, 句子头不含 '$') */
static const char *field(const char *s, int n)
{
    if (n == 0) {
        return s;
    }
    int i = 0;
    while (i < n) {
        s = strchr(s, ',');
        if (!s) return NULL;
        s++;
        i++;
    }
    return s;
}

/* 拷贝第 n 个字段到 dst(不含逗号), 供文本类字段使用 */
static void field_copy(const char *line, int n, char *dst, size_t cap)
{
    const char *p = field(line, n);
    if (!p) { dst[0] = '\0'; return; }
    size_t i = 0;
    while (p[i] != '\0' && p[i] != ',' && i + 1 < cap) {
        dst[i] = p[i];
        i++;
    }
    dst[i] = '\0';
}

/* 去掉首尾空白(就地整理) */
static void trim(char *s)
{
    char *start = s;
    while (*start == ' ' || *start == '\t') start++;

    char *e = start + strlen(start);
    while (e > start && (e[-1] == ' ' || e[-1] == '\t' || e[-1] == '\r')) e--;
    *e = '\0';

    if (start != s) {
        memmove(s, start, (size_t)(e - start) + 1);
    }
}

static int two_digits(const char *p)
{
    if (!p || !isdigit((unsigned char)p[0]) || !isdigit((unsigned char)p[1])) {
        return -1;
    }
    return (p[0] - '0') * 10 + (p[1] - '0');
}

static int hex_val(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

/* 某字段是否为可解析的数字 */
static bool num_field(const char *line, int idx, double *out)
{
    const char *p = field(line, idx);
    if (!p || !*p) return false;
    if (!isdigit((unsigned char)*p) && *p != '-' && *p != '+') return false;
    *out = strtod(p, NULL);
    return true;
}

/* 大小写不敏感子串查找 */
static char lower_char(char c)
{
    if (c >= 'A' && c <= 'Z') return (char)(c - 'A' + 'a');
    return c;
}

static bool contains_ci(const char *hay, const char *needle)
{
    if (!*needle) return true;
    for (; *hay; hay++) {
        const char *h = hay, *n = needle;
        while (*n && *h && lower_char(*h) == lower_char(*n)) { h++; n++; }
        if (!*n) return true;
    }
    return false;
}

/* NMEA 时间 hhmmss[.sss] -> gps_info_t 时间字段 */
static bool nmea_time(const char *p, gps_info_t *g)
{
    int hh, mm, ss;
    if (!p) return false;
    hh = two_digits(p);
    mm = two_digits(p + 2);
    ss = two_digits(p + 4);
    if (hh < 0 || mm < 0 || ss < 0) return false;
    if (hh > 23 || mm > 59 || ss > 60) return false;

    uint16_t ms = 0;
    if (p[6] == '.') {
        unsigned m1 = 0, m2 = 0, m3 = 0;
        if (isdigit((unsigned char)p[7])) m1 = (unsigned)(p[7] - '0');
        if (isdigit((unsigned char)p[8])) m2 = (unsigned)(p[8] - '0');
        if (isdigit((unsigned char)p[9])) m3 = (unsigned)(p[9] - '0');
        ms = (uint16_t)(m1 * 100 + m2 * 10 + m3);
        if (ms > 999) ms = 999;
    }

    g->hour        = (uint8_t)hh;
    g->minute      = (uint8_t)mm;
    g->second      = (uint8_t)ss;
    g->millisecond = ms;
    return true;
}

/* RMC 日期 ddmmyy */
static bool nmea_date_ddmmyy(const char *p, gps_info_t *g)
{
    int dd, mo, yy;
    if (!p) return false;
    dd = two_digits(p);
    mo = two_digits(p + 2);
    yy = two_digits(p + 4);
    if (dd < 1 || dd > 31 || mo < 1 || mo > 12 || yy < 0) return false;

    g->day   = (uint8_t)dd;
    g->month = (uint8_t)mo;
    g->year  = (uint16_t)(2000 + yy);
    return true;
}

/* ZDA 日期 d,m,yyyy (年 4 位, 跨世纪可靠) */
static bool nmea_date_zda(const char *d, const char *m, const char *y, gps_info_t *g)
{
    int dd, mo, yy;
    if (!d || !m || !y) return false;
    if (!isdigit((unsigned char)*d) || !isdigit((unsigned char)*m) ||
        !isdigit((unsigned char)*y)) {
        return false;
    }
    dd = atoi(d);
    mo = atoi(m);
    yy = atoi(y);
    if (dd < 1 || dd > 31 || mo < 1 || mo > 12 || yy < 2000 || yy > 2099) return false;

    g->day   = (uint8_t)dd;
    g->month = (uint8_t)mo;
    g->year  = (uint16_t)yy;
    return true;
}

/* 度分格式(ddmm.mmmm + N/S/E/W) -> 十进制度 */
static bool nmea_latlon(const char *val, char hemi, double *deg)
{
    if (!val || !isdigit((unsigned char)val[0])) return false;
    double v = strtod(val, NULL);
    double d = (int)(v / 100.0);
    double m = v - d * 100.0;
    if (m >= 60.0) return false;

    double out = d + m / 60.0;
    if (hemi == 'S' || hemi == 'W') out = -out;
    *deg = out;
    return true;
}

/* ============================================================
 * 快照读写
 * ============================================================ */

/* 把 ISR 记录的 PPS 信息合并进快照 */
static void pack_pps(gps_info_t *g)
{
    portENTER_CRITICAL(&s_pps_lock);
    g->pps_seq     = s_pps_seq;
    g->pps_edge_us = s_pps_last_us;
    portEXIT_CRITICAL(&s_pps_lock);
}

void gps_get_info(gps_info_t *out)
{
    if (!out) return;
    if (s_mutex) xSemaphoreTake(s_mutex, portMAX_DELAY);
    *out = s_info;
    if (s_mutex) xSemaphoreGive(s_mutex);
    pack_pps(out);   /* PPS 原子信息单独更新 */
}

/* 提交新快照并通知数据回调 */
static void commit(const gps_info_t *g)
{
    if (s_mutex) xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_info = *g;
    if (s_mutex) xSemaphoreGive(s_mutex);

    gps_data_callback_t cb;
    void *arg;
    if (s_mutex) xSemaphoreTake(s_mutex, portMAX_DELAY);
    cb  = s_data_cb;
    arg = s_data_cb_arg;
    if (s_mutex) xSemaphoreGive(s_mutex);

    if (cb) {
        gps_info_t copy;
        gps_get_info(&copy);
        cb(&copy, arg);
    }
}

/* ============================================================
 * NMEA 语句解析
 * ============================================================ */

/* ---- GGA: 时间/位置/质量/卫星数/HDOP/海拔 ---- */
static void handle_gga(const char *line)
{
    gps_info_t g;
    gps_get_info(&g);
    bool changed = false;

    const char *p = field(line, 1);
    if (p && nmea_time(p, &g)) {
        g.time_valid = true;
        changed = true;
    }

    double q;
    if (num_field(line, 6, &q)) {
        g.fix_quality = (uint8_t)q;
        g.fix_valid   = (q > 0);
        changed = true;
    }

    double sv;
    if (num_field(line, 7, &sv)) {
        g.satellites = (uint8_t)sv;
        changed = true;
    }

    double hdop;
    if (num_field(line, 8, &hdop)) {
        g.hdop = (float)hdop;
    }

    double alt;
    if (num_field(line, 9, &alt)) {
        g.altitude_m = (float)alt;
    }

    /* 位置 */
    const char *lat_p = field(line, 2);
    const char *ns    = field(line, 3);
    const char *lon_p = field(line, 4);
    const char *ew    = field(line, 5);
    double lat = 0, lon = 0;
    if (ns && ew && nmea_latlon(lat_p, *ns, &lat) && nmea_latlon(lon_p, *ew, &lon)) {
        g.latitude  = lat;
        g.longitude = lon;
        changed = true;
    }

    if (changed) {
        commit(&g);
    }
}

/* ---- RMC: 时间/状态/位置/速度/航向/日期/模式 ---- */
static void handle_rmc(const char *line)
{
    gps_info_t g;
    gps_get_info(&g);
    bool changed = false;

    const char *p = field(line, 1);
    if (p && nmea_time(p, &g)) {
        g.time_valid = true;
        changed = true;
    }

    const char *st = field(line, 2);
    bool active = (st && *st == 'A');
    g.fix_valid = active;
    changed = true;

    if (active) {
        const char *lat_p = field(line, 3);
        const char *ns    = field(line, 4);
        const char *lon_p = field(line, 5);
        const char *ew    = field(line, 6);
        double lat = 0, lon = 0;
        if (ns && ew && nmea_latlon(lat_p, *ns, &lat) && nmea_latlon(lon_p, *ew, &lon)) {
            g.latitude  = lat;
            g.longitude = lon;
            changed = true;
        }

        double spd, crs;
        if (num_field(line, 7, &spd)) {
            g.speed_kmh = (float)(spd * 1.852);   /* 节 -> km/h */
        }
        if (num_field(line, 8, &crs)) {
            g.course_deg = (float)crs;
        }
    }

    p = field(line, 9);
    if (p && nmea_date_ddmmyy(p, &g)) {
        g.date_valid = true;
        changed = true;
    }

    /* 模式指示符(位置 idx12, 如 'A' 自主定位) */
    p = field(line, 12);
    if (p && *p && (*p >= 'A' && *p <= 'Z')) {
        g.mode_ind = *p;
    }

    if (changed) {
        commit(&g);
    }
}

/* ---- GLL: 位置 + 时间 + 状态 + 模式 ---- */
static void handle_gll(const char *line)
{
    gps_info_t g;
    gps_get_info(&g);
    bool changed = false;

    const char *p = field(line, 5);
    if (p && nmea_time(p, &g)) {
        g.time_valid = true;
        changed = true;
    }

    const char *st = field(line, 6);
    bool active = (st && *st == 'A');
    if (active) {
        const char *lat_p = field(line, 1);
        const char *ns    = field(line, 2);
        const char *lon_p = field(line, 3);
        const char *ew    = field(line, 4);
        double lat = 0, lon = 0;
        if (ns && ew && nmea_latlon(lat_p, *ns, &lat) && nmea_latlon(lon_p, *ew, &lon)) {
            g.latitude  = lat;
            g.longitude = lon;
            changed = true;
        }
    }
    g.fix_valid = active;
    changed = true;

    p = field(line, 7);
    if (p && *p && (*p >= 'A' && *p <= 'Z')) {
        g.mode_ind = *p;
    }

    if (changed) {
        commit(&g);
    }
}

/* ---- GSA: 每星座一条, 含 PRN 列表 / PDOP / HDOP / VDOP / 系统ID ---- */
static void handle_gsa(const char *line)
{
    gps_info_t g;
    gps_get_info(&g);

    double fixv = 0;
    if (num_field(line, 2, &fixv)) {
        uint8_t f = (uint8_t)fixv;
        if (f > g.fix_type) g.fix_type = f;   /* 取最高(3D) */
    }

    /* PRN 列表(idx3~14) */
    uint8_t sv[GPS_SV_PER_SYS];
    uint8_t cnt = 0;
    double v;
    for (int k = 3; k <= 14 && cnt < GPS_SV_PER_SYS; k++) {
        if (num_field(line, k, &v) && v > 0) {
            uint8_t prn = (uint8_t)v;
            if (prn > 0) sv[cnt++] = prn;
        }
    }

    double dv;
    if (num_field(line, 15, &dv)) g.pdop = (float)dv;
    if (num_field(line, 16, &dv)) g.hdop = (float)dv;
    if (num_field(line, 17, &dv)) g.vdop = (float)dv;

    /* 系统 ID(idx18); 旧式单星座语句可能缺省, 由 talker 推测 */
    uint8_t sysid = 0;
    if (num_field(line, 18, &dv)) {
        sysid = (uint8_t)dv;
    }
    if (sysid == 0) {
        if (line[0] == 'G' && line[1] == 'P') sysid = 1;        /* GPS */
        else if (line[0] == 'G' && line[1] == 'L') sysid = 2;   /* GLONASS */
        else if (line[0] == 'G' && line[1] == 'A') sysid = 3;   /* Galileo */
        else if (line[0] == 'G' && line[1] == 'B') sysid = 4;   /* BDS */
    }

    int slot = -1;
    if (sysid != 0) {
        for (int i = 0; i < (int)g.sys_count; i++) {
            if (g.sys[i].system_id == sysid) { slot = i; break; }
        }
        if (slot < 0 && g.sys_count < GPS_SYS_MAX) {
            slot = (int)g.sys_count++;
        }
        if (slot >= 0) {
            g.sys[slot].system_id = sysid;
            g.sys[slot].fix_type  = (uint8_t)fixv;
            g.sys[slot].sv_count  = cnt;
            memcpy(g.sys[slot].sv, sv, cnt);
        }
    }

    commit(&g);
}

/* ---- VTG: 真航向 / 速度(节与 km/h) ---- */
static void handle_vtg(const char *line)
{
    gps_info_t g;
    gps_get_info(&g);

    double v;
    if (num_field(line, 1, &v)) {
        g.course_deg = (float)v;   /* 对地真航向 */
    }
    if (num_field(line, 7, &v)) {
        g.speed_kmh = (float)v;    /* km/h 字段优先 */
    } else if (num_field(line, 5, &v)) {
        g.speed_kmh = (float)(v * 1.852);   /* 节字段兜底 */
    }

    commit(&g);
}

/* ---- ZDA: 时间 + 完整年月日(年 4 位) + 时区 ---- */
static void handle_zda(const char *line)
{
    gps_info_t g;
    gps_get_info(&g);
    bool changed = false;

    const char *p = field(line, 1);
    if (p && nmea_time(p, &g)) {
        g.time_valid = true;
        changed = true;
    }
    if (nmea_date_zda(field(line, 2), field(line, 3), field(line, 4), &g)) {
        g.date_valid = true;
        changed = true;
    }

    if (changed) {
        commit(&g);
    }
}

/* ---- TXT: 厂商文本, 解析天线告警 ---- */
static void handle_txt(const char *line)
{
    char text[40];
    field_copy(line, 4, text, sizeof(text));
    trim(text);

    if (!contains_ci(text, "antenna")) {
        return;   /* 非天线消息, 忽略 */
    }

    gps_antenna_state_t st = GPS_ANT_UNKNOWN;
    if (contains_ci(text, "open")) {
        st = GPS_ANT_OPEN;
    } else if (contains_ci(text, "short")) {
        st = GPS_ANT_SHORT;
    } else if (contains_ci(text, "ok") || contains_ci(text, "good") ||
               contains_ci(text, "normal")) {
        st = GPS_ANT_OK;
    }

    gps_info_t g;
    gps_get_info(&g);
    if (st != GPS_ANT_UNKNOWN) {
        g.antenna = st;
        strncpy(g.antenna_text, text, sizeof(g.antenna_text) - 1);
        g.antenna_text[sizeof(g.antenna_text) - 1] = '\0';
        commit(&g);
    }
}

/* 校验并分发一条完整句子(不含 '$' 与末尾换行) */
static void parse_sentence(char *line)
{
    /* 校验和: 对 '*' 前所有字节 XOR, 与 '*' 后 2 位十六进制比较 */
    char *star = strchr(line, '*');
    if (star) {
        uint8_t cs = 0;
        for (char *c = line; c < star; c++) {
            cs ^= (uint8_t)*c;
        }
        int hi = hex_val(star[1]);
        int lo = hex_val(star[2]);
        if (hi < 0 || lo < 0) return;
        if ((uint8_t)((hi << 4) | lo) != cs) return;   /* 校验失败丢弃 */
        *star = '\0';
    }

    if (strlen(line) < 5) return;

    /* 句子类型: 取 talker(2位)之后的三位语句码, 兼容 GN/GP/GL 等前缀 */
    const char *c = line + 2;
    if      (c[0] == 'G' && c[1] == 'G' && c[2] == 'A') handle_gga(line);
    else if (c[0] == 'R' && c[1] == 'M' && c[2] == 'C') handle_rmc(line);
    else if (c[0] == 'G' && c[1] == 'L' && c[2] == 'L') handle_gll(line);
    else if (c[0] == 'G' && c[1] == 'S' && c[2] == 'A') handle_gsa(line);
    else if (c[0] == 'V' && c[1] == 'T' && c[2] == 'G') handle_vtg(line);
    else if (c[0] == 'Z' && c[1] == 'D' && c[2] == 'A') handle_zda(line);
    else if (c[0] == 'T' && c[1] == 'X' && c[2] == 'T') handle_txt(line);
}

/* 单字节喂入行状态机 */
static void feed_byte(uint8_t b)
{
    if (b == '\r') return;

    if (b == '\n') {
        if (!s_line_overflow && s_line_len > 0) {
            s_line[s_line_len] = '\0';
            parse_sentence(s_line);
        }
        s_line_len = 0;
        s_line_overflow = false;
        return;
    }

    if (b == '$') {            /* 新句子开始(容错) */
        s_line_len = 0;
        s_line_overflow = false;
        return;
    }

    if (!s_line_overflow) {
        if (s_line_len < GPS_LINE_MAX - 1) {
            s_line[s_line_len++] = (char)b;
        } else {
            s_line_overflow = true;   /* 超长, 丢弃到行尾 */
        }
    }
}

static void gps_parse_task(void *arg)
{
    (void)arg;
    uint8_t buf[128];
    for (;;) {
        int n = uart_read_bytes(GPS_UART_NUM, buf, sizeof(buf), pdMS_TO_TICKS(200));
        if (n <= 0) continue;
        for (int i = 0; i < n; i++) {
            feed_byte(buf[i]);
        }
    }
}

/* ============================================================
 * PPS 秒脉冲: 中断 -> 队列 -> 回调
 * ============================================================ */

static void IRAM_ATTR pps_isr_handler(void *arg)
{
    (void)arg;
    BaseType_t hpw = pdFALSE;
    int64_t ts = esp_timer_get_time();   /* ISR 里抓取尽量精确的时间戳 */

    portENTER_CRITICAL_ISR(&s_pps_lock);
    s_pps_last_us = ts;
    s_pps_seq++;
    uint32_t seq = s_pps_seq;
    portEXIT_CRITICAL_ISR(&s_pps_lock);

    pps_evt_t ev = { ts, seq };
    if (s_pps_q) {
        xQueueSendFromISR(s_pps_q, &ev, &hpw);
    }
    if (hpw) {
        portYIELD_FROM_ISR(hpw);
    }
}

static void pps_init_gpio(void)
{
    if (PPS_GPIO == GPIO_NUM_NC) {
        return;
    }

    gpio_config_t cfg = {
        .pin_bit_mask  = (1ULL << PPS_GPIO),
        .mode          = GPIO_MODE_INPUT,
        .pull_up_en    = GPIO_PULLUP_DISABLE,
        .pull_down_en  = GPIO_PULLDOWN_ENABLE,   /* 未定位时默认低电平 */
        .intr_type     = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&cfg));

    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(err);
    }

    ESP_ERROR_CHECK(gpio_isr_handler_add(PPS_GPIO, pps_isr_handler, NULL));
    ESP_ERROR_CHECK(gpio_set_intr_type(PPS_GPIO, GPIO_INTR_POSEDGE));
    ESP_ERROR_CHECK(gpio_intr_enable(PPS_GPIO));
}

static void pps_task(void *arg)
{
    (void)arg;
    pps_evt_t ev;
    for (;;) {
        if (!xQueueReceive(s_pps_q, &ev, portMAX_DELAY)) {
            continue;
        }

        gps_pps_callback_t cb;
        void *cb_arg;
        if (s_mutex) xSemaphoreTake(s_mutex, portMAX_DELAY);
        cb     = s_pps_cb;
        cb_arg = s_pps_cb_arg;
        if (s_mutex) xSemaphoreGive(s_mutex);

        gps_info_t snap;
        gps_get_info(&snap);

        if (cb) {
            cb(ev.ts_us, ev.seq, &snap, cb_arg);
        }
    }
}

/* ============================================================
 * 对外 API
 * ============================================================ */

esp_err_t gps_set_pps_callback(gps_pps_callback_t cb, void *arg)
{
    if (s_mutex) xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_pps_cb     = cb;
    s_pps_cb_arg = arg;
    if (s_mutex) xSemaphoreGive(s_mutex);
    return ESP_OK;
}

esp_err_t gps_set_data_callback(gps_data_callback_t cb, void *arg)
{
    if (s_mutex) xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_data_cb     = cb;
    s_data_cb_arg = arg;
    if (s_mutex) xSemaphoreGive(s_mutex);
    return ESP_OK;
}

esp_err_t gps_init(void)
{
    if (s_started) {
        return ESP_OK;
    }

    /* 1) 互斥锁 / 队列 */
    if (!s_mutex) s_mutex = xSemaphoreCreateMutex();
    if (!s_pps_q) s_pps_q = xQueueCreate(8, sizeof(pps_evt_t));
    if (!s_mutex || !s_pps_q) {
        ESP_LOGE(TAG, "alloc failed");
        return ESP_ERR_NO_MEM;
    }

    /* 2) UART1 */
    const uart_config_t ucfg = {
        .baud_rate  = GPS_UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    esp_err_t err = uart_driver_install(GPS_UART_NUM, GPS_RX_BUF_SIZE, 0, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(err));
        return err;
    }
    err = uart_param_config(GPS_UART_NUM, &ucfg);
    if (err != ESP_OK) {
        uart_driver_delete(GPS_UART_NUM);
        return err;
    }
    err = uart_set_pin(GPS_UART_NUM, GNSS_TXD, GNSS_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        uart_driver_delete(GPS_UART_NUM);
        return err;
    }

    /* 3) PPS GPIO 中断(可配 PPS_GPIO = GPIO_NUM_NC 关闭) */
    pps_init_gpio();

    /* 4) 任务 */
    xTaskCreate(gps_parse_task, "gps_parse", GPS_PARSE_TASK_STACK, NULL,
                GPS_PARSE_TASK_PRIO, &s_parse_handle);
    if (PPS_GPIO != GPIO_NUM_NC) {
        xTaskCreate(pps_task, "gps_pps", GPS_PPS_TASK_STACK, NULL,
                    GPS_PPS_TASK_PRIO, &s_pps_handle);
    }

    s_started = true;
    ESP_LOGI(TAG, "GNSS ready: UART%d @%d baud, TX=GPIO%d RX=GPIO%d PPS=GPIO%d",
             GPS_UART_NUM, GPS_UART_BAUD, GNSS_TXD, GNSS_RXD, PPS_GPIO);
    return ESP_OK;
}

esp_err_t gps_deinit(void)
{
    if (!s_started) {
        return ESP_OK;
    }

    if (PPS_GPIO != GPIO_NUM_NC) {
        gpio_intr_disable(PPS_GPIO);
        if (s_pps_handle) vTaskDelete(s_pps_handle);
        s_pps_handle = NULL;
    }
    if (s_parse_handle) vTaskDelete(s_parse_handle);
    s_parse_handle = NULL;

    uart_driver_delete(GPS_UART_NUM);

    if (s_pps_q) {
        vQueueDelete(s_pps_q);
        s_pps_q = NULL;
    }
    s_started = false;
    ESP_LOGI(TAG, "GNSS stopped");
    return ESP_OK;
}

void gps_log_info(void)
{
    gps_info_t g;
    gps_get_info(&g);

    const char *ant = "unknown";
    if (g.antenna == GPS_ANT_OK) ant = "OK";
    else if (g.antenna == GPS_ANT_OPEN) ant = "OPEN";
    else if (g.antenna == GPS_ANT_SHORT) ant = "SHORT";

    ESP_LOGI(TAG, "fix=%s qual=%d type=%d sv=%u pos=(%.6f, %.6f) dops=%.1f/%.1f/%.1f alt=%.1fm ant=%s(%s)",
             g.fix_valid ? "Y" : "N", g.fix_quality, g.fix_type, g.satellites,
             g.latitude, g.longitude, g.pdop, g.hdop, g.vdop, g.altitude_m,
             ant, g.antenna_text);
    ESP_LOGI(TAG, "utc=%04u-%02u-%02u %02u:%02u:%02u.%03u valid(t=%d d=%d) spd=%.1fkm/h crs=%.1fdeg",
             g.year, g.month, g.day, g.hour, g.minute, g.second, g.millisecond,
             g.time_valid, g.date_valid, g.speed_kmh, g.course_deg);
    for (int i = 0; i < (int)g.sys_count; i++) {
        ESP_LOGI(TAG, "  sys%d(GSA fix=%d): %u sats", g.sys[i].system_id,
                 g.sys[i].fix_type, g.sys[i].sv_count);
    }
    ESP_LOGI(TAG, "pps_seq=%lu last_edge_us=%lld", (unsigned long)g.pps_seq,
             (long long)g.pps_edge_us);
}
