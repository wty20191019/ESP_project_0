/* 入口：FT8/FT4 应用(模块 ft8_app) + GNSS/显示屏(GPS + LCD_task)
 *
 * 说明：
 *  - ft8_app 为单任务控制器：持续解码，并在每个协议槽按配置定时发射。
 *  - LCD_task 循环显示 GPS(GNSS) 信息: 时间/经纬度/各星座卫星/信号。
 *
 * 接线约定：
 *   LCD : SCK=GPIO10, MOSI=GPIO11, CS=GPIO42, DC=GPIO4, RST=GPIO5, BLK=GPIO6
 *   GNSS: UART1 TX=GPIO17 -> RX, RX=GPIO18 <- TX, PPS=GPIO9
 *   LED : GPIO48
 */
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "ft8_app.h"
#include "led.h"
#include "lcd.h"
#include "gps.h"
#include "key.h"

#define TAG "main"

/* FT8 应用全局配置：以引用交给 ft8_app(需保持有效)；
 * 下面 gps_time_task 会持续把 GPS UTC 时间/日期/PPS 填入 cfg.gps */
static ft8_app_config_t cfg;

/* 屏幕显示本地时间所用时区(北京 = UTC+8)。UTC 直接显示可改为 0 */
#define LCD_TZ_HOUR    8
#define LCD_ROW_H      16                  /* 8x16 ASCII 行高 */
#define LCD_MAX_ROWS   (LCD_H / LCD_ROW_H) /* 128x160 屏最多 10 行 */

/* ================= LCD 显示辅助 ================= */

/* 整行清底并显示文本(自动截断到 16 列) */
static void lcd_row(int row, uint16_t color, const char *fmt, ...)
{
    if (row < 0 || row >= LCD_MAX_ROWS) return;

    char b[20];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(b, sizeof(b), fmt, ap);
    va_end(ap);
    if (n < 0) n = 0;
    if (n > (int)sizeof(b) - 1) n = (int)sizeof(b) - 1;
    if (n > 16) n = 16;                     /* 每行最多 16 个 8x16 字符 */
    b[n] = '\0';

    uint16_t y = (uint16_t)(row * LCD_ROW_H);
    LCD_Fill(0, y, LCD_W, y + LCD_ROW_H, BLACK);
    LCD_ShowString(0, y, (const uint8_t *)b, color, BLACK, 16, 0);
}

/* 星座中文含义 -> 短名称 */
static const char *sys_name(uint8_t id)
{
    switch (id) {
    case 1: return "GPS";
    case 2: return "GLO";
    case 3: return "GAL";
    case 4: return "BDS";
    case 5: return "QZS";
    case 6: return "SBA";
    default: return "SYS";
    }
}

static uint8_t days_in_month(uint16_t y, uint8_t m)
{
    static const uint8_t d[13] = { 0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
    if (m == 2 && ((y % 4 == 0 && y % 100 != 0) || y % 400 == 0)) return 29;
    return (m >= 1 && m <= 12) ? d[m] : 31;
}

/* UTC 时间 + 时区偏移 -> 本地时间(结构体拷贝, 不修改 GPS 快照) */
static void gps_local(const gps_info_t *g, int tz_h, gps_info_t *out)
{
    *out = *g;
    if (!g->time_valid) return;

    int h = g->hour + tz_h;
    int carry = 0;
    while (h >= 24) { h -= 24; carry++; }
    while (h < 0)   { h += 24; carry--; }
    out->hour = (uint8_t)h;

    if (carry) {
        uint16_t y = g->year;
        uint8_t  m = g->month;
        uint8_t  d = g->day;
        for (int i = 0; i < abs(carry); i++) {
            if (carry > 0) {                 /* 跨日进位 */
                d++;
                if (d > days_in_month(y, m)) { d = 1; if (++m > 12) { m = 1; y++; } }
            } else {                         /* 跨日借位(负偏移用) */
                if (d > 1) d--;
                else {
                    if (m == 1) { m = 12; y--; } else m--;
                    d = days_in_month(y, m);
                }
            }
        }
        out->year  = y;
        out->month = m;
        out->day   = d;
    }
}

/* 把一组卫星 PRN 分到若干行(每行 ≤16 字符)显示 */
static void print_prns(int *row, uint8_t cnt, const uint8_t *sv)
{
    char buf[20];
    buf[0] = '\0';
    for (int i = 0; i < cnt && *row < LCD_MAX_ROWS; i++) {
        char tok[8];
        snprintf(tok, sizeof(tok), "%u ", sv[i]);
        if (strlen(buf) + strlen(tok) >= 16) {
            lcd_row((*row)++, WHITE, "%s", buf);
            buf[0] = '\0';
        }
        strncat(buf, tok, sizeof(buf) - 1 - strlen(buf));
    }
    if (buf[0] && *row < LCD_MAX_ROWS) {
        lcd_row((*row)++, WHITE, "%s", buf);
    }
}

/* 汇总各星座卫星数, 输出如 "GPS5 GLO2 BDS5" (最多 3 个) */
static void build_sys_counts(const gps_info_t *g, char *out, size_t cap)
{
    out[0] = '\0';
    int shown = 0;
    for (int i = 0; i < (int)g->sys_count && shown < 3; i++) {
        char one[12];
        snprintf(one, sizeof(one), "%s%u ", sys_name(g->sys[i].system_id),
                 g->sys[i].sv_count);
        if (strlen(out) + strlen(one) < cap) {
            strcat(out, one);
            shown++;
        }
    }
}

/* ================= 三页显示 ================= */

/* 第 0 页: 时间 / 日期 / 经纬 / 卫星汇总 / 基本状态 */
static void draw_page_info(const gps_info_t *src)
{
    gps_info_t g;
    gps_local(src, LCD_TZ_HOUR, &g);

    if (g.time_valid) {
        lcd_row(0, YELLOW, "%02d:%02d:%02d", g.hour, g.minute, g.second);
    } else {
        lcd_row(0, YELLOW, "--:--:--");
    }
    lcd_row(1, WHITE, "%04d-%02d-%02d", g.year, g.month, g.day);

    if (g.fix_valid) {
        lcd_row(2, GREEN, "N %0.6f", g.latitude);
        lcd_row(3, GREEN, "E %0.6f", g.longitude);
        char cnt[24];
        build_sys_counts(&g, cnt, sizeof(cnt));
        lcd_row(4, CYAN, "%s", cnt[0] ? cnt : "sat waiting");
        lcd_row(5, CYAN, "SVs%2u HDOP%.1f", g.satellites, g.hdop);
        lcd_row(6, WHITE, "spd%.1f crs%.1f", g.speed_kmh, g.course_deg);
    } else {
        lcd_row(2, GRAY, "N ---------");
        lcd_row(3, GRAY, "E ---------");
        lcd_row(4, GRAY, "sat waiting");
        lcd_row(5, GRAY, "SVs  0  HDOP --");
        lcd_row(6, GRAY, "fix searching");
    }

    /* 天线状态 + 定位质量 */
    if (g.antenna == GPS_ANT_OK) {
        lcd_row(7, WHITE, "ANT OK   Q%d", g.fix_quality);
    } else if (g.antenna == GPS_ANT_OPEN) {
        lcd_row(7, RED, "ANT OPEN!");
    } else if (g.antenna == GPS_ANT_SHORT) {
        lcd_row(7, RED, "ANT SHORT!");
    } else {
        lcd_row(7, GRAY, "ANT --    Q%d", g.fix_quality);
    }
    lcd_row(8, WHITE, "PPS#%lu", (unsigned long)g.pps_seq);
    lcd_row(9, WHITE, "ALT%0.1fm  T%d", g.altitude_m, g.fix_type);
}

/* 第 1 页: 各星座参与卫星(PRN)明细 */
static void draw_page_sat(const gps_info_t *g)
{
    lcd_row(0, CYAN, "SATELLITES");
    if (g->sys_count == 0) {
        lcd_row(1, GRAY, "waiting GSA...");
        return;
    }
    int row = 1;
    for (int i = 0; i < (int)g->sys_count && row < LCD_MAX_ROWS; i++) {
        lcd_row(row++, GREEN, "%s %uSV %dD",
                sys_name(g->sys[i].system_id), g->sys[i].sv_count,
                g->sys[i].fix_type);
        print_prns(&row, g->sys[i].sv_count, g->sys[i].sv);
    }
}

/* 第 2 页: 信号 / DOP / 其它状态 */
static void draw_page_signal(gps_info_t g)
{
    lcd_row(0, CYAN, "SIGNAL");

    if (!g.fix_valid) {
        lcd_row(1, GRAY, "NO FIX");
        return;
    }

    /* 用卫星数+HDOP 折算信号格数 */
    int sig;
    if (g.satellites >= 8)      sig = (g.hdop < 1.5f) ? 4 : (g.hdop < 2.5f) ? 3 : 2;
    else if (g.satellites >= 4) sig = 2;
    else                        sig = 1;

    char bar[8];
    for (int i = 0; i < 4; i++) bar[i] = (i < sig) ? '#' : '.';
    bar[4] = '\0';
    lcd_row(1, (sig >= 3) ? GREEN : (sig == 2 ? YELLOW : RED), "SIG [%s]", bar);
    lcd_row(2, WHITE, "SVs%2u  HDOP%.1f", g.satellites, g.hdop);
    lcd_row(3, WHITE, "PDOP%.1f VDOP%.1f", g.pdop, g.vdop);
    lcd_row(4, WHITE, "Q%d T%d M%c", g.fix_quality, g.fix_type,
            g.mode_ind ? g.mode_ind : '-');
    lcd_row(5, WHITE, "ALT%0.1fm", g.altitude_m);

    if (g.antenna == GPS_ANT_OK) {
        lcd_row(6, GREEN, "%s", g.antenna_text[0] ? g.antenna_text : "ANT OK");
    } else if (g.antenna == GPS_ANT_OPEN) {
        lcd_row(6, RED, "%s", g.antenna_text[0] ? g.antenna_text : "ANT OPEN");
    } else if (g.antenna == GPS_ANT_SHORT) {
        lcd_row(6, RED, "%s", g.antenna_text[0] ? g.antenna_text : "ANT SHORT");
    } else {
        lcd_row(6, GRAY, "ANT UNKNOWN");
    }
    lcd_row(7, WHITE, "PPS#%lu", (unsigned long)g.pps_seq);
    lcd_row(8, WHITE, "spd%0.1f crs%0.1f", g.speed_kmh, g.course_deg);
}

/* ================= 瀑布页(页 3) ================= */

/* ===== 瀑布伪彩: 256 级 LUT, 在色标结点间线性插值(冷→热), 噪声底以下为黑 ===== */
#define WF_NOISE_FLOOR   30     /* v<120(约 -60dB 以下)视为背景噪声 -> 黑 */

typedef struct {
    uint8_t v;          /* 色标所在幅度 */
    uint8_t r, g, b;    /* 该点颜色(8bit/通道) */
} wf_stop_t;

static const wf_stop_t s_wf_stops[] = {
    { WF_NOISE_FLOOR,   0,   0,  80 },   /* 黑蓝底 */
    { 140,   0,  40, 255 },   /* 蓝 */
    { 160,   0, 255, 255 },   /* 青 */
    { 180,  40, 255,  20 },   /* 绿 */
    { 200, 255, 255,  40 },   /* 黄 */
    { 220, 255, 140,   0 },   /* 橙 */
    { 240, 255,  30,   0 },   /* 红 */
    { 255, 255, 255, 255 },   /* 白(削波) */
};

static uint16_t s_wf_lut[256];
static bool s_wf_lut_ok = false;

static inline uint16_t wf_rgb565(int r, int g, int b)
{
    if (r < 0) r = 0;
    if (r > 255) r = 255;
    if (g < 0) g = 0;
    if (g > 255) g = 255;
    if (b < 0) b = 0;
    if (b > 255) b = 255;
    return (uint16_t)(((uint16_t)(r >> 3) << 11) | ((uint16_t)(g >> 2) << 5) | (uint16_t)(b >> 3));
}

static void wf_lut_build(void)
{
    const int N = (int)(sizeof(s_wf_stops) / sizeof(s_wf_stops[0]));
    for (int v = 0; v < 256; v++)
        s_wf_lut[v] = 0;                                  /* 先全黑(含噪声底) */

    for (int i = 0; i < N - 1; i++) {
        int v0 = s_wf_stops[i].v;
        int v1 = s_wf_stops[i + 1].v;
        for (int v = v0; v < v1 && v < 256; v++) {
            float t = (float)(v - v0) / (float)(v1 - v0);
            int r = s_wf_stops[i].r + (int)(t * (s_wf_stops[i + 1].r - s_wf_stops[i].r));
            int g = s_wf_stops[i].g + (int)(t * (s_wf_stops[i + 1].g - s_wf_stops[i].g));
            int b = s_wf_stops[i].b + (int)(t * (s_wf_stops[i + 1].b - s_wf_stops[i].b));
            s_wf_lut[v] = wf_rgb565(r, g, b);
        }
    }
    s_wf_lut[s_wf_stops[N - 1].v] = wf_rgb565(s_wf_stops[N - 1].r, s_wf_stops[N - 1].g, s_wf_stops[N - 1].b);
    s_wf_lut_ok = true;
}

/* 幅度字节(v 0..255, 大致对应 -120..0dB) -> RGB565(平滑渐变) */
static uint16_t wf_color(uint8_t v)
{
    if (!s_wf_lut_ok) wf_lut_build();
    return s_wf_lut[v];
}

/* 第 3 页: 瀑布图。最新一行贴屏幕底部, 历史向上滚动。
 * 数据源: ft8_app RX 任务每收到一个符号块, 就把该块功率谱压缩成一行
 * 128 点写入环形快照(见 ft8_app.h 的 ft8_wf_snap_t / ft8_wf_snap)。 */
static void draw_page_fft(void)
{
    char buf[40];
    snprintf(buf, sizeof(buf), "WF %.3g-%.3gk",
             (double)cfg.rx_f_min / 1000.0, (double)cfg.rx_f_max / 1000.0);
    lcd_row(0, CYAN, "%s", buf);

    const ft8_wf_snap_t *wf = ft8_wf_snap();
    if (wf == NULL || wf->seq == 0) {
        lcd_row(4, GRAY, "NO DATA");
        lcd_row(5, GRAY, "wait rx...");
        return;
    }

    const uint32_t seq = wf->seq;              /* 一次性读取, 容忍极轻微跨核竞态 */
    const uint32_t put = wf->put;
    uint32_t n = (seq < FT8_WF_ROWS) ? seq : FT8_WF_ROWS;
    uint32_t oldest = ((put - n) % FT8_WF_ROWS + FT8_WF_ROWS) % FT8_WF_ROWS;

    /* 每一行对应一个符号时段; 逐行按颜色游程水平填充, 减少绘图调用 */
    for (uint32_t k = 0; k < n; k++) {
        const uint8_t *row = wf->rows[(oldest + k) % FT8_WF_ROWS];
        int y = LCD_H - 1 - (int)(n - 1 - k);

        int x0 = 0;
        uint16_t cur = wf_color(row[0]);
        for (int x = 1; x <= FT8_WF_COLS; x++) {
            uint16_t c = (x < FT8_WF_COLS) ? wf_color(row[x]) : (uint16_t)(cur ^ 0x100);
            if (c != cur) {
                if (cur != BLACK && x > x0)          /* 底色由 LCD_Clear 负责, 黑段可跳过 */
                    LCD_Fill((uint16_t)x0, (uint16_t)y, (uint16_t)x, (uint16_t)(y + 1), cur);
                x0 = x;
                cur = c;
            }
        }
    }
}

/* ================= LCD 主任务 ================= */
#define LCD_PAGE_NUM    4                  /* 页数: 0=信息 1=卫星 2=信号 3=瀑布 */
static volatile int s_lcd_page = 0;        /* 当前显示页, 由按键回调修改 */

static void LCD_task(void *arg)
{
    static uint8_t res = 0;
    if (res == 0)
    {
        LCD_Init();
        ESP_LOGI(TAG, "LCD任务启动");
        gps_init();
        res = 1;
    }

    for (;;)
    {
        gps_info_t g;
        gps_get_info(&g);

        int page = s_lcd_page;              /* 手动按键翻页, 不再自动轮换 */

        LCD_Clear(BLACK);
        if (page == 0)      draw_page_info(&g);
        else if (page == 1) draw_page_sat(&g);
        else if (page == 2) draw_page_signal(g);
        else if (page == 3) draw_page_fft();

        LCD_Flush();                       /* 画完一整帧后一次性推送 */

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/* ================= RGB LED 呼吸灯 ================= */
static void rgb_led_task(void *arg)
{
    static uint8_t res = 0;
    if (res == 0)
    {
        led_init();
        ESP_LOGI(TAG, "RGB LED 呼吸任务启动");
        res = 1;
    }

    for (;;)
    {
        /* 红色呼吸 */
        for (int i = 0; i <= 255; i += 5) { led_set_rgb(i, 0, 0); vTaskDelay(pdMS_TO_TICKS(5)); }
        for (int i = 255; i > 0; i -= 5)  { led_set_rgb(i, 0, 0); vTaskDelay(pdMS_TO_TICKS(5)); }
        /* 绿色呼吸 */
        for (int i = 0; i <= 255; i += 5) { led_set_rgb(0, i, 0); vTaskDelay(pdMS_TO_TICKS(5)); }
        for (int i = 255; i > 0; i -= 5)  { led_set_rgb(0, i, 0); vTaskDelay(pdMS_TO_TICKS(5)); }
        /* 蓝色呼吸 */
        for (int i = 0; i <= 255; i += 5) { led_set_rgb(0, 0, i); vTaskDelay(pdMS_TO_TICKS(5)); }
        for (int i = 255; i > 0; i -= 5)  { led_set_rgb(0, 0, i); vTaskDelay(pdMS_TO_TICKS(5)); }
    }
}

/* ================= GPS UTC 时间 -> ft8_app 配置 =================
 * GPS 模块自身已有后台 NMEA 解析任务(见 components/BSP/GPS)，这里只是把
 * gps_get_info() 快照里的 UTC 时间/日期/PPS 拷贝进全局 cfg.gps，
 * 供 ft8_app 在启用 gps_utc_enable 时做 UTC 时隙对齐。 */
static void gps_time_task(void *arg)
{
    bool reported = false;
    for (;;)
    {
        gps_info_t g;
        gps_get_info(&g);

        if (g.time_valid && g.date_valid)
        {
            cfg.gps.valid       = true;
            cfg.gps.year        = g.year;
            cfg.gps.month       = g.month;
            cfg.gps.day         = g.day;
            cfg.gps.hour        = g.hour;
            cfg.gps.minute      = g.minute;
            cfg.gps.second      = g.second;
            cfg.gps.millisecond = g.millisecond;
            cfg.gps.pps_seq     = g.pps_seq;          /* 没接 PPS 时为 0 */
            cfg.gps.pps_edge_us = g.pps_edge_us;
            if (!reported)
            {
                ESP_LOGI(TAG, "GPS UTC 就绪: %04u-%02u-%02u %02u:%02u:%02u.%03u PPS#%s",
                         g.year, g.month, g.day, g.hour, g.minute, g.second,
                         (unsigned)g.millisecond,
                         g.pps_seq ? "有" : "无");
                reported = true;
            }
        }
        else
        {
            reported = false;   /* 失锁后再定位时重新上报 */
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

static void my_key_callback(key_id_t key_id, key_event_t event, void *user_data)
{
    /* 单击: 上/下/左/右 翻页, 中间键回首页 */
    if (event == KEY_EVENT_CLICK)
    {
        ESP_LOGI("APP", "按键 %d 单击", key_id);
        switch (key_id)
        {
        case KEY_ID_UP:
        case KEY_ID_LEFT:
            s_lcd_page = (s_lcd_page + LCD_PAGE_NUM - 1) % LCD_PAGE_NUM;
            ESP_LOGI("APP", "按键翻页 -> %d", s_lcd_page);
            break;
        case KEY_ID_DOWN:
        case KEY_ID_RIGHT:
            s_lcd_page = (s_lcd_page + 1) % LCD_PAGE_NUM;
            ESP_LOGI("APP", "按键翻页 -> %d", s_lcd_page);
            break;
        case KEY_ID_MID:
            s_lcd_page = 0;
            ESP_LOGI("APP", "按键返回首页");
            break;
        default:
            break;
        }
    }
    else if (event == KEY_EVENT_LONG_PRESS)
    {
        ESP_LOGI("APP", "按键 %d 长按 ", key_id);
    }
}

void app_main(void)
{
    key_init(my_key_callback, NULL);
    gps_init();                       /* 启动 GPS NMEA/PPS 后台(幂等) */

    /* ====== FT8/FT4 配置示例 ====== */
    ft8_app_config_default(&cfg);

    cfg.protocol            = FTX_PROTOCOL_FT8;             /* FTX_PROTOCOL_FT4 切到 FT4(7.5s 时隙) */
    cfg.tx_enable           = true;                         /* 参与发射(仅在选中奇偶时隙) */
    cfg.rx_enable           = true;                         /* 持续解码 */
    cfg.utc_enable          = true;                         /* 总开关：按 UTC 对齐时隙 */
    cfg.gps_utc_enable      = true;                         /* 选择使用 GPS 的 UTC 时间/日期对齐 */
    cfg.gps_use_pps         = false;                        /* true=用 PPS 精对齐; false=不用PPS, NMEA粗对齐 */
    cfg.tx_slot_parity      = 0;                            /* 0=偶时隙发 / 1=奇时隙发，自动与对端交替 */
    cfg.tx_delay_ms         = 500;                          /* 本台时隙内再延时发射 */
    cfg.rx_parse_ms         = 100;                          /* 每个时隙结束前静默期(ms)，用于整窗解析 */    
    snprintf(cfg.callsign, sizeof(cfg.callsign), "BG7ZJW"); // 本机呼号
    snprintf(cfg.grid,     sizeof(cfg.grid),     "JO70");   // 本机网格
    cfg.tx.type             = FT8_APP_MSG_CQ;               /* 第几类消息: CQ / CALL / REPORT / R_REPORT / RRR / RR73 / 73 */
    cfg.tx.cq_modifier[0]   = '\0';                         /* "DX"/"WW"/"TEST"... 仅 CQ 类用 */
    cfg.tx.call_to[0]       = '\0';                         /* 目标呼号(类型 2~6 用)，如 "BG5ABC" */
    cfg.tx.rst_db           = -12;                          /* 信号报告 dB(类型 3/4 用)：REPORT 发 -12，R_REPORT 发 R-12 */
    cfg.audio_freq_hz       = 1500.0f;                      /* 音频中心(8-GFSK tone0) */
    cfg.audio_level         = 0.80f;                        /* 发射电平 */ /*!< 发射幅度 0~1，防削波建议 ≤0.9 */
    cfg.rx_f_max            = 3000.0f;                      /* 解码频率上限 */
    cfg.rx_f_min            = 50.0f;                        /* 解码频率下限 */
    cfg.max_candidates      = 50;                           /*每时隙解码耗时 ≈ 候选数(max_candidates) × 每个候选迭代数(ldpc_iterations) × 单次迭代成本*/
    cfg.ldpc_iterations     = 25;

    /* WM8978 编解码器参数(对应原硬编码的 ADDA(1,1)/Input(1,1,0)/MIC40/Output(1,0)/I2S(2,0)/HP(50,50)/SPK40，
     * 默认已一致，这里仅示例按需修改) */
    cfg.codec.mic_gain = 40;                        // MIC 增益 0~63(-12~+35.25dB，0.75dB/步)
    cfg.codec.hp_vol_l = 50;                        // L声道耳机音量 (0~63)
    cfg.codec.hp_vol_r = cfg.codec.hp_vol_l;        // R声道耳机音量 (0~63)
    cfg.codec.spk_vol  = 0;                         // 音响音量 (0~63)

    cfg.qso.enable           = true ;      // 启用自动 QSO 引擎(启用才建队, RX 解码无队时不产生额外开销)
    cfg.qso.cq_mode          = true ;      // 主叫模式: 自动 CQ, 完成 QSO 后自动收下一个
    cfg.qso.max_retries      = 4;          // 超过此次数仍无进展则放弃该台
    cfg.qso.target_callsign[0] = '\0';     // 应答模式: 只应答此呼号, 空则应答所有陌生 CQ 台

    //true 
    //false

    //启动任务============================================================================================
    /* 搬运 GPS UTC 时间/日期/PPS 进 cfg.gps(供 ft8_app UTC 对齐，先启动让它尽早喂数据) */
    xTaskCreatePinnedToCore(gps_time_task, "gps_utc", 4096, NULL, 5, NULL, 1);

    if (ft8_app_start(&cfg) != ESP_OK)
    {
        ESP_LOGE(TAG, "ft8_app 启动失败");
        return;
    }

    /* 运行中热切换发射内容：直接改配置字段即可(ft8_app 按引用使用 cfg)，下一本台时隙生效：
     *     cfg.tx.type = FT8_APP_MSG_RR73;
     *     snprintf(cfg.tx.call_to, sizeof(cfg.tx.call_to), "BG5ABC");
     */

    xTaskCreatePinnedToCore(LCD_task, "LCD", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(rgb_led_task, "rgb_led", 2048, NULL, 1, NULL, 0);


}