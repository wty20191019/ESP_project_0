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
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "ft8_app.h"
#include "qso_log.h"
#include "wm8978.h"
#include "led.h"
#include "lcd.h"
#include "gps.h"
#include "key.h"

#define TAG "main"

/* FT8 应用全局配置：以引用交给 ft8_app(需保持有效)；
 * 下面 gps_time_task 会持续把 GPS UTC 时间/日期/PPS 填入 cfg.gps */
static ft8_app_config_t cfg;
static volatile bool s_cfg_dirty = false;   /* 配置被修改, 待写入 cfg.txt */

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

/* 指定字号与 y 坐标的整行绘制(size: 12/16/24/32), 自动清底并截断 */
static void lcd_line(int y, uint16_t color, uint8_t size, const char *fmt, ...)
{
    if (y < 0 || y + size > LCD_H) return;

    char b[64];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(b, sizeof(b), fmt, ap);
    va_end(ap);
    if (n < 0) n = 0;
    if (n > (int)sizeof(b) - 1) n = (int)sizeof(b) - 1;
    int maxc = LCD_W / (size / 2);          /* 该字号最多字符数 */
    if (n > maxc) n = maxc;
    b[n] = '\0';

    LCD_Fill(0, (uint16_t)y, LCD_W, (uint16_t)(y + size), BLACK);
    LCD_ShowString(0, (uint16_t)y, (const uint8_t *)b, color, BLACK, size, 0);
}

/* 同 lcd_line, 但可指定背景色(用于选中反色) */
static void lcd_line_bg(int y, uint8_t size, uint16_t fc, uint16_t bc, const char *fmt, ...)
{
    if (y < 0 || y + size > LCD_H) return;

    char b[64];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(b, sizeof(b), fmt, ap);
    va_end(ap);
    if (n < 0) n = 0;
    if (n > (int)sizeof(b) - 1) n = (int)sizeof(b) - 1;
    int maxc = LCD_W / (size / 2);
    if (n > maxc) n = maxc;
    b[n] = '\0';

    LCD_Fill(0, (uint16_t)y, LCD_W, (uint16_t)(y + size), bc);
    LCD_ShowString(0, (uint16_t)y, (const uint8_t *)b, fc, bc, size, 0);
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

/* ================= 显示 ================= */

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

/* ===== 瀑布伪彩: 256 级 LUT(结点间线性插值), 输入是逐帧归一化后的 0..255 强度 ===== */
typedef struct {
    uint8_t v;          /* 色标位置(归一化 0..255) */
    uint8_t r, g, b;    /* 该点颜色(8bit/通道) */
} wf_stop_t;

static const wf_stop_t s_wf_stops[] = {
    {   0,   8,   0,  30 },   /* 近黑(噪声底) */
    {  70,   0,   0, 220 },   /* 深蓝 */
    { 110,   0,  60, 255 },   /* 蓝 */
    { 150,   0, 235, 255 },   /* 青 */
    { 185,  30, 255,  60 },   /* 绿 */
    { 215, 255, 255,  30 },   /* 黄 */
    { 240, 255,  90,   0 },   /* 橙 */
    { 255, 255, 255, 255 },   /* 白(顶) */
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

/* 归一化强度(0..255) -> RGB565(平滑渐变) */
static uint16_t wf_color(uint8_t rel)
{
    if (!s_wf_lut_ok) wf_lut_build();
    return s_wf_lut[rel];
}

static const char *const s_msg_opts[] = { "CQ", "CALL", "REPORT", "R_REPORT", "RRR", "RR73", "73" };

/* ================= 瀑布绘制(通用区域) =================
 * 把 wf 快照最近 h 行画到 x=0..127, y0..y0+h-1, 最新行贴底部;
 * 逐帧直方图自适应拉伸 + 颜色游程填充。 */
static void draw_waterfall(int y0, int h)
{
    const ft8_wf_snap_t *wf = ft8_wf_snap();
    if (wf == NULL || wf->seq == 0 || h <= 0) return;

    const uint32_t put = wf->put;
    uint32_t avail = (wf->seq < FT8_WF_ROWS) ? wf->seq : FT8_WF_ROWS;
    uint32_t n = (avail < (uint32_t)h) ? avail : (uint32_t)h;
    if (n == 0) return;
    uint32_t oldest = ((put - n) % FT8_WF_ROWS + FT8_WF_ROWS) % FT8_WF_ROWS;

    uint16_t hist[256] = { 0 };
    for (uint32_t k = 0; k < n; k++) {
        const uint8_t *row = wf->rows[(oldest + k) % FT8_WF_ROWS];
        for (int x = 0; x < FT8_WF_COLS; x++) hist[row[x]]++;
    }
    const uint32_t total = n * FT8_WF_COLS;
    int acc = 0, lo = 0;
    for (int i = 0; i < 256; i++) { acc += hist[i]; if (acc * 2 >= (int)total) { lo = i; break; } }
    int hi = 0;
    for (int i = 255; i >= 0; i--) if (hist[i]) { hi = i; break; }
    int span = hi - lo;
    if (span < 24) span = 24;

    uint16_t cmap[256];
    for (int i = 0; i < 256; i++) {
        int rel = (int)(((int64_t)i - lo) * 256 / span);
        if (rel < 0) rel = 0;
        if (rel > 255) rel = 255;
        cmap[i] = wf_color((uint8_t)rel);
    }

    for (uint32_t k = 0; k < n; k++) {
        const uint8_t *row = wf->rows[(oldest + k) % FT8_WF_ROWS];
        int y = y0 + h - 1 - (int)(n - 1 - k);
        int x0 = 0;
        uint16_t cur = cmap[row[0]];
        for (int x = 1; x <= FT8_WF_COLS; x++) {
            uint16_t c = (x < FT8_WF_COLS) ? cmap[row[x]] : (uint16_t)(cur ^ 0x100);
            if (c != cur) {
                if (cur != BLACK && x > x0)
                    LCD_Fill((uint16_t)x0, (uint16_t)y, (uint16_t)x, (uint16_t)(y + 1), cur);
                x0 = x;
                cur = c;
            }
        }
    }
}

/* ================= 页 0 主操作页(仿 FT8CN, 小字): 状态 + 瀑布(含AF红线) + 发射设置 ================= */
#define MAIN_WF_Y   12
#define MAIN_WF_H   76                  /* y12..87 */
#define MAIN_SCALE_Y (MAIN_WF_Y + MAIN_WF_H)   /* 88 */
#define MAIN_ITEM_Y  (MAIN_SCALE_Y + 12)       /* 100, 4 项 * 12px = 48 -> 148 */

#define MAIN_ITEM_N 4
static int s_main_sel = 0;              /* 0=TX开关 1=步数 2=时隙 3=音频频率 */

static void main_adjust(int dir)
{
    switch (s_main_sel) {
    case 0:
        cfg.tx_enable = !cfg.tx_enable;
        break;
    case 1: {
        int v = (int)cfg.tx.type + dir;
        if (v < 0) v = 6;
        if (v > 6) v = 0;
        cfg.tx.type = (ft8_app_msg_type_t)v;
        break;
    }
    case 2:
        cfg.tx_slot_parity = (cfg.tx_slot_parity & 1) ? 0 : 1;
        break;
    case 3: {
        float f = cfg.audio_freq_hz + dir * 50.0f;
        if (f < 100.0f)  f = 100.0f;
        if (f > 3000.0f) f = 3000.0f;
        cfg.audio_freq_hz = f;
        break;
    }
    default: break;
    }
    s_cfg_dirty = true;
}

static void draw_page_main(void)
{
    bool tx = ft8_app_tx_busy();

    /* 状态栏(小字): 频率 MHz + UTC; 右侧 TX(红)/RX(绿) */
    if (cfg.qso_freq_mhz > 0.0f)
        lcd_line(0, WHITE, 12, "%.3f %02u%02u%02u", (double)cfg.qso_freq_mhz,
                 cfg.gps.valid ? cfg.gps.hour : 0,
                 cfg.gps.valid ? cfg.gps.minute : 0,
                 cfg.gps.valid ? cfg.gps.second : 0);
    else
        lcd_line(0, WHITE, 12, "%s %02u%02u%02u", cfg.band[0] ? cfg.band : "---",
                 cfg.gps.valid ? cfg.gps.hour : 0,
                 cfg.gps.valid ? cfg.gps.minute : 0,
                 cfg.gps.valid ? cfg.gps.second : 0);
    LCD_ShowString((uint16_t)(LCD_W - 12), 0, (const uint8_t *)(tx ? "TX" : "RX"),
                   tx ? RED : GREEN, BLACK, 12, 0);

    /* 瀑布 + 音频频率红线定位 */
    draw_waterfall(MAIN_WF_Y, MAIN_WF_H);
    if (cfg.rx_f_max > cfg.rx_f_min) {
        int x = (int)((cfg.audio_freq_hz - cfg.rx_f_min) * (LCD_W - 1) /
                      (cfg.rx_f_max - cfg.rx_f_min));
        if (x < 0) x = 0;
        if (x > LCD_W - 1) x = LCD_W - 1;
        LCD_Fill((uint16_t)x, MAIN_WF_Y, (uint16_t)(x + 1),
                 (uint16_t)(MAIN_WF_Y + MAIN_WF_H), RED);
    }

    /* 频率刻度 */
    lcd_line(MAIN_SCALE_Y, GRAY, 12, "%.3gk-%.3gk",
             (double)cfg.rx_f_min / 1000.0, (double)cfg.rx_f_max / 1000.0);

    /* 设置项(小字, 选中反色) */
    int typ = (int)cfg.tx.type;
    if (typ < 0 || typ > 6) typ = 0;
    for (int i = 0; i < MAIN_ITEM_N; i++) {
        int y = MAIN_ITEM_Y + i * 12;
        bool sel = (i == s_main_sel);
        switch (i) {
        case 0:
            if (sel) lcd_line_bg(y, 12, BLACK, YELLOW, "TX  %s", cfg.tx_enable ? "ON" : "OFF");
            else     lcd_line(y, cfg.tx_enable ? GREEN : GRAY, 12, "TX  %s", cfg.tx_enable ? "ON" : "OFF");
            break;
        case 1:
            if (sel) lcd_line_bg(y, 12, BLACK, YELLOW, "STEP %s", s_msg_opts[typ]);
            else     lcd_line(y, WHITE, 12, "STEP %s", s_msg_opts[typ]);
            break;
        case 2:
            if (sel) lcd_line_bg(y, 12, BLACK, YELLOW, "SLOT %s", (cfg.tx_slot_parity & 1) ? "odd" : "even");
            else     lcd_line(y, WHITE, 12, "SLOT %s", (cfg.tx_slot_parity & 1) ? "odd" : "even");
            break;
        case 3:
            if (sel) lcd_line_bg(y, 12, BLACK, YELLOW, "AF   %.0fHz", cfg.audio_freq_hz);
            else     lcd_line(y, WHITE, 12, "AF   %.0fHz", cfg.audio_freq_hz);
            break;
        default: break;
        }
    }

    /* 当前步数对应的发射内容预览 */
    {
        int y = MAIN_ITEM_Y + MAIN_ITEM_N * 12;   /* 148 */
        const char *to = cfg.tx.call_to[0] ? cfg.tx.call_to : "-";
        switch (typ) {
        case FT8_APP_MSG_CQ:
            if (cfg.tx.cq_modifier[0])
                lcd_line(y, CYAN, 12, "CQ %s %s %s", cfg.tx.cq_modifier, cfg.callsign, cfg.grid);
            else
                lcd_line(y, CYAN, 12, "CQ %s %s", cfg.callsign, cfg.grid);
            break;
        case FT8_APP_MSG_CALL:
            lcd_line(y, CYAN, 12, "%s %s %s", to, cfg.callsign, cfg.grid);
            break;
        case FT8_APP_MSG_REPORT:
            lcd_line(y, CYAN, 12, "%s %s %+d", to, cfg.callsign, cfg.tx.rst_db);
            break;
        case FT8_APP_MSG_R_REPORT:
            lcd_line(y, CYAN, 12, "%s %s R%+d", to, cfg.callsign, cfg.tx.rst_db);
            break;
        case FT8_APP_MSG_RRR:
            lcd_line(y, CYAN, 12, "%s %s RRR", to, cfg.callsign);
            break;
        case FT8_APP_MSG_RR73:
            lcd_line(y, CYAN, 12, "%s %s RR73", to, cfg.callsign);
            break;
        case FT8_APP_MSG_73:
            lcd_line(y, CYAN, 12, "%s %s 73", to, cfg.callsign);
            break;
        default: break;
        }
    }
}

/* ================= 页 1 解码列表(小字, 呼号一行 + 其他一行, 新信息在底部) ================= */
static int s_rx_sel = 0;        /* 选中项(时间序: 0=最旧, cnt-1=最新) */
static int s_rx_last_cnt = 0;
#define DEC_ROWS 6              /* 一屏 6 条(每条 2 行小字, 24px) */

/* CQ=黄, 呼我=绿, 收尾=青, 其它=白 */
static uint16_t rx_msg_color(const char *text)
{
    if (strncmp(text, "CQ", 2) == 0) return YELLOW;
    if (strstr(text, "RR73") || strstr(text, "RRR") || strstr(text, " 73")) return CYAN;
    if (cfg.callsign[0] && strncmp(text, cfg.callsign, strlen(cfg.callsign)) == 0) return GREEN;
    return WHITE;
}

static int decode_count(void)
{
    const ft8_rx_log_t *log = ft8_rx_log();
    if (log == NULL || log->seq == 0) return 0;
    return (log->seq < FT8_RX_MSG_MAX) ? (int)log->seq : FT8_RX_MSG_MAX;
}

/* 时间序第 k 项(0=最旧) -> 环形下标 */
static const ft8_rx_msg_t *decode_at(int k, int cnt)
{
    const ft8_rx_log_t *log = ft8_rx_log();
    if (log == NULL || k < 0 || k >= cnt) return NULL;
    uint32_t idx = (log->put - (uint32_t)cnt + (uint32_t)k + FT8_RX_MSG_MAX * 2) % FT8_RX_MSG_MAX;
    return &log->msgs[idx];
}

static void draw_page_decode(void)
{
    bool tx = ft8_app_tx_busy();
    lcd_row(0, CYAN, "DECODE");
    LCD_ShowString((uint16_t)(LCD_W - 16), 0, (const uint8_t *)(tx ? "TX" : "RX"),
                   tx ? RED : GREEN, BLACK, 16, 0);

    int cnt = decode_count();

    /* 新消息到达且原先停在最新 -> 跟随到最新 */
    if (cnt != s_rx_last_cnt) {
        if (s_rx_sel >= s_rx_last_cnt - 1) s_rx_sel = cnt - 1;
        s_rx_last_cnt = cnt;
    }
    if (s_rx_sel > cnt - 1) s_rx_sel = cnt - 1;
    if (s_rx_sel < 0) s_rx_sel = 0;

    /* 选中项居中滚动 */
    int top = s_rx_sel - (DEC_ROWS / 2);
    if (top < 0) top = 0;
    if (top > cnt - DEC_ROWS) top = cnt - DEC_ROWS;
    if (top < 0) top = 0;

    for (int r = 0; r < DEC_ROWS; r++) {
        int k = top + r;
        int y = 16 + r * 24;                 /* 每条 24px: 文本 12 + DT/SNR 12 */
        const ft8_rx_msg_t *m = decode_at(k, cnt);
        if (m == NULL) {
            lcd_line(y, BLACK, 12, "");
            lcd_line(y + 12, BLACK, 12, "");
            continue;
        }
        uint16_t col = rx_msg_color(m->text);
        bool sel = (k == s_rx_sel);

        /* 第 1 行: 解码文本 */
        if (sel) lcd_line_bg(y, 12, BLACK, YELLOW, "%s", m->text);
        else     lcd_line(y, col, 12, "%s", m->text);

        /* 第 2 行: DT(时间差)  SNR(信号) */
        if (isfinite(m->snr_db)) {
            if (sel) lcd_line_bg(y + 12, 12, BLACK, YELLOW, "%+4.1f %+3.0f", m->dt_s, m->snr_db);
            else     lcd_line(y + 12, GRAY, 12, "%+4.1f %+3.0f", m->dt_s, m->snr_db);
        } else {
            if (sel) lcd_line_bg(y + 12, 12, BLACK, YELLOW, "%+4.1f  --", m->dt_s);
            else     lcd_line(y + 12, GRAY, 12, "%+4.1f  --", m->dt_s);
        }
    }
}

/* ================= 页 2 GPS 概览(信息+卫星+信号合并) ================= */
static void draw_page_gps(const gps_info_t *g)
{
    lcd_row(0, CYAN, "GPS %s", g->fix_valid ? "FIX" : "NOFIX");

    if (g->time_valid)
        lcd_row(1, WHITE, "%04u-%02u-%02u %02u:%02u:%02u",
                g->year, g->month, g->day, g->hour, g->minute, g->second);
    else
        lcd_row(1, GRAY, "date --");

    if (g->fix_valid) {
        lcd_row(2, GREEN, "N %0.5f", g->latitude);
        lcd_row(3, GREEN, "E %0.5f", g->longitude);
    } else {
        lcd_row(2, GRAY, "N -----");
        lcd_row(3, GRAY, "E -----");
    }

    lcd_row(4, WHITE, "SVs%2u HDOP%.1f", g->satellites, g->hdop);
    lcd_row(5, WHITE, "PDOP%.1f VDOP%.1f", g->pdop, g->vdop);
    lcd_row(6, WHITE, "ALT%0.0fm %0.1fkm/h", g->altitude_m, g->speed_kmh);
    lcd_row(7, WHITE, "CRS%0.0f Q%d T%d M%c", g->course_deg, g->fix_quality, g->fix_type,
            g->mode_ind ? g->mode_ind : '-');

    if (g->antenna == GPS_ANT_OK)         lcd_row(8, GREEN, "ANT OK");
    else if (g->antenna == GPS_ANT_OPEN)  lcd_row(8, RED, "ANT OPEN!");
    else if (g->antenna == GPS_ANT_SHORT) lcd_row(8, RED, "ANT SHORT!");
    else                                  lcd_row(8, GRAY, "ANT --");

    lcd_row(9, WHITE, "PPS#%lu", (unsigned long)g->pps_seq);
}

/* ================= 页 3 日志(呼号/网格/信号/频率, 每条 2 行; 中键看原文详情) ================= */
static int s_log_sel = 9999;       /* 选中记录(0=最旧), 初始贴最新 */
static int s_log_top = 9999;       /* 列表顶部记录 */
static int s_log_hscroll = 0;
static int s_log_tick = 0;
static int s_log_detail = 0;

#define LOG_VIS_ROWS 6                 /* 一屏 6 条(每条 2 行小字) */

static void draw_page_log(void)
{
    /* 约每 1s 重新读取一次文件尾部(读失败保留旧内容) */
    if ((s_log_tick++ % 50) == 0) qso_log_tail(0);   /* 0 = 读取全部记录 */

    int n = qso_log_tail_count();

    if (s_log_detail) {                 /* 详情: 显示原始 ADIF 行, 可横向滚动 */
        lcd_row(0, CYAN, "DETAIL %d/%d", s_log_sel + 1, n);
        const char *raw = qso_log_tail_line(s_log_sel);
        if (raw) {
            int len = (int)strlen(raw);
            int off = s_log_hscroll;
            if (off > len) off = len;
            lcd_row(1, WHITE, "%s", raw + off);
        }
        lcd_row(9, GRAY, "MID: back");
        return;
    }

    lcd_row(0, CYAN, "LOG %d", n);
    if (n == 0) { lcd_row(4, GRAY, "empty / usb busy"); return; }

    if (s_log_sel > n - 1) s_log_sel = n - 1;
    if (s_log_sel < 0) s_log_sel = 0;
    if (s_log_top > n - 1) s_log_top = n - 1;
    if (s_log_top < 0) s_log_top = 0;
    if (s_log_sel < s_log_top) s_log_top = s_log_sel;
    if (s_log_sel > s_log_top + (LOG_VIS_ROWS - 1)) s_log_top = s_log_sel - (LOG_VIS_ROWS - 1);

    for (int i = 0; i < LOG_VIS_ROWS; i++) {
        int k = s_log_top + i;
        int y = 16 + i * 24;                 /* 每条 24px: 两行 12px 小字 */
        const qso_log_sum_t *s = (k < n) ? qso_log_tail_summary(k) : NULL;
        if (s == NULL) { LCD_Fill(0, (uint16_t)y, LCD_W, (uint16_t)(y + 24), BLACK); continue; }
        uint16_t col = (k == s_log_sel) ? YELLOW : WHITE;
        lcd_line(y,      col, 12, "%s", s->call);                     /* 呼号 */
        lcd_line(y + 12, col, 12, "%s %s %s", s->grid, s->rst, s->freq); /* 网格 信号 频率 */
    }
}

/* ================= 页 7: 配置设置(可编辑 cfg 所有主要项) ================= */
typedef enum { CI_BOOL, CI_INT, CI_U8, CI_U32, CI_FLOAT, CI_STR, CI_ENUM } ci_type_t;

typedef struct {
    const char *name;
    ci_type_t   type;
    void       *ptr;
    float       vmin, vmax, step;
    int         len;                 /* 字符串缓冲长度 */
    const char *const *opts;         /* 枚举选项名 */
    int         nopts;
    int         dec;                 /* 浮点显示小数位 */
} ci_item_t;

static const char *const s_proto_opts[] = { "FT4", "FT8" };

static const ci_item_t s_ci[] = {
    { "callsign", CI_STR,   cfg.callsign,              0,0,0, sizeof(cfg.callsign), NULL, 0, 0 },
    { "grid",     CI_STR,   cfg.grid,                  0,0,0, sizeof(cfg.grid),     NULL, 0, 0 },
    { "band",     CI_STR,   cfg.band,                  0,0,0, sizeof(cfg.band),     NULL, 0, 0 },
    { "freq_MHz", CI_FLOAT, &cfg.qso_freq_mhz,       0.1f, 60.0f, 0.001f, 0, NULL, 0, 6 },
    { "protocol", CI_ENUM,  &cfg.protocol,             0,0,0, 0, s_proto_opts, 2, 0 },
    { "tx_en",    CI_BOOL,  &cfg.tx_enable,            0,0,0, 0, NULL, 0, 0 },
    { "rx_en",    CI_BOOL,  &cfg.rx_enable,            0,0,0, 0, NULL, 0, 0 },
    { "usb_msc",  CI_BOOL,  &cfg.usb_mount_enable,     0,0,0, 0, NULL, 0, 0 },
    { "utc_en",   CI_BOOL,  &cfg.utc_enable,           0,0,0, 0, NULL, 0, 0 },
    { "gps_utc",  CI_BOOL,  &cfg.gps_utc_enable,       0,0,0, 0, NULL, 0, 0 },
    { "gps_pps",  CI_BOOL,  &cfg.gps_use_pps,          0,0,0, 0, NULL, 0, 0 },
    { "slot_par", CI_INT,   &cfg.tx_slot_parity,       0, 1, 1, 0, NULL, 0, 0 },
    { "delay_ms", CI_U32,   &cfg.tx_delay_ms,          0, 10000, 50, 0, NULL, 0, 0 },
    { "msg_type", CI_ENUM,  &cfg.tx.type,              0,0,0, 0, s_msg_opts, 7, 0 },
    { "call_to",  CI_STR,   cfg.tx.call_to,            0,0,0, sizeof(cfg.tx.call_to), NULL, 0, 0 },
    { "cq_mod",   CI_STR,   cfg.tx.cq_modifier,        0,0,0, sizeof(cfg.tx.cq_modifier), NULL, 0, 0 },
    { "rst_db",   CI_INT,   &cfg.tx.rst_db,          -30, 30, 1, 0, NULL, 0, 0 },
    { "af_Hz",    CI_FLOAT, &cfg.audio_freq_hz,        50, 3000, 10, 0, NULL, 0, 0 },
    { "af_lvl",   CI_FLOAT, &cfg.audio_level,          0, 1, 0.05f, 0, NULL, 0, 2 },
    { "hp_vol",   CI_U8,    &cfg.codec.hp_vol_l,       0, 63, 1, 0, NULL, 0, 0 },
    { "rx_fmin",  CI_FLOAT, &cfg.rx_f_min,             0, 3000, 50, 0, NULL, 0, 0 },
    { "rx_fmax",  CI_FLOAT, &cfg.rx_f_max,           100, 5000, 50, 0, NULL, 0, 0 },
    { "cand",     CI_INT,   &cfg.max_candidates,       1, 128, 5, 0, NULL, 0, 0 },
    { "ldpc_it",  CI_INT,   &cfg.ldpc_iterations,      1, 100, 5, 0, NULL, 0, 0 },//
    { "parse_ms", CI_U32,   &cfg.rx_parse_ms,          0, 5000, 100, 0, NULL, 0, 0 },
    { "qso_en",   CI_BOOL,  &cfg.qso.enable,           0,0,0, 0, NULL, 0, 0 },
    { "qso_cq",   CI_BOOL,  &cfg.qso.cq_mode,          0,0,0, 0, NULL, 0, 0 },
    { "qso_rty",  CI_INT,   &cfg.qso.max_retries,      1, 60, 1, 0, NULL, 0, 0 },
    { "qso_to",   CI_STR,   cfg.qso.target_callsign,   0,0,0, sizeof(cfg.qso.target_callsign), NULL, 0, 0 },
};
#define CI_N ((int)(sizeof(s_ci) / sizeof(s_ci[0])))
#define CI_ROWS 12                 /* 小字 12px, 一屏 12 行 */

static int s_ci_sel = 0;
static int s_ci_scroll = 0;
static int s_ci_cursor = 0;
static bool s_ci_edit = false;      /* 是否处于修改状态(中键进/出) */
/* 字符集: 空格 + 大小写字母 + 数字 + 符号 / \ ' ? . - = + _ */
static const char s_ci_charset[] =
    " ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789/\\'?.-=+_";

/* 把配置项当前值格式化成字符串(与显示一致), 返回长度 */
static int ci_value_str(const ci_item_t *it, char *out, size_t cap)
{
    out[0] = '\0';
    switch (it->type) {
    case CI_BOOL:  snprintf(out, cap, "%s", *(bool *)it->ptr ? "Y" : "N"); break;
    case CI_INT:   snprintf(out, cap, "%d", *(int *)it->ptr); break;
    case CI_U8:    snprintf(out, cap, "%u", (unsigned)*(uint8_t *)it->ptr); break;
    case CI_U32:   snprintf(out, cap, "%lu", (unsigned long)*(uint32_t *)it->ptr); break;
    case CI_FLOAT: snprintf(out, cap, "%.*f", it->dec, (double)*(float *)it->ptr); break;
    case CI_STR:   snprintf(out, cap, "%s", (char *)it->ptr); break;
    case CI_ENUM: {
        int v = *(int *)it->ptr;
        if (v < 0 || v >= it->nopts) v = 0;
        snprintf(out, cap, "%s", it->opts[v]);
        break;
    }
    default: break;
    }
    return (int)strlen(out);
}

/* 修改"当前光标位": 数值按该位的权值加减(带进位/借位), 字符串换字符, 枚举/布尔整体切换 */
static void ci_edit_step(const ci_item_t *it, int dir)
{
    if (it->type == CI_BOOL) { *(bool *)it->ptr = !*(bool *)it->ptr; return; }
    if (it->type == CI_ENUM) {
        int v = *(int *)it->ptr + dir;
        if (v < 0) v = it->nopts - 1;
        if (v >= it->nopts) v = 0;
        *(int *)it->ptr = v;
        return;
    }

    char v[24];
    int len = ci_value_str(it, v, sizeof(v));
    if (len <= 0) return;
    int pos = s_ci_cursor;
    if (pos < 0) pos = 0;
    if (pos >= len) pos = len - 1;
    char c = v[pos];

    if (it->type == CI_STR) {
        const char *p = strchr(s_ci_charset, c);
        int idx = p ? (int)(p - s_ci_charset) : 0;
        int setn = (int)strlen(s_ci_charset);
        idx = (idx + dir + setn) % setn;
        char *s = (char *)it->ptr;
        s[pos] = s_ci_charset[idx];
        if (s[pos] && pos + 1 < it->len && s[pos + 1] == '\0') s[pos + 1] = '\0';
        return;
    }

    /* 数值: 符号位切换正负 */
    if (c == '-' || c == '+') {
        if (it->type == CI_INT) {
            int x = -*(int *)it->ptr;
            if (x < (int)it->vmin) x = (int)it->vmin;
            if (x > (int)it->vmax) x = (int)it->vmax;
            *(int *)it->ptr = x;
        } else if (it->type == CI_FLOAT) {
            float x = -*(float *)it->ptr;
            if (x < it->vmin) x = it->vmin;
            if (x > it->vmax) x = it->vmax;
            *(float *)it->ptr = x;
        }
        return;
    }
    if (c < '0' || c > '9') return;

    /* 计算光标所在位的权值: 整数位=10^n, 小数位=10^-(距小数点位数); 小数点不参与 */
    int dot = -1;
    for (int i = 0; i < len; i++) if (v[i] == '.') { dot = i; break; }
    if (dot >= 0 && pos == dot) return;

    double place;
    if (dot < 0 || pos < dot) {
        int end = (dot < 0) ? len : dot;
        int cnt = 0;
        for (int i = pos + 1; i < end; i++)
            if (v[i] >= '0' && v[i] <= '9') cnt++;
        place = pow(10.0, cnt);
    } else {
        place = pow(10.0, -(double)(pos - dot));
    }

    if (it->type == CI_INT) {
        long x = (long)*(int *)it->ptr + (long)(dir * place);
        if (x < (long)it->vmin) x = (long)it->vmin;
        if (x > (long)it->vmax) x = (long)it->vmax;
        *(int *)it->ptr = (int)x;
    } else if (it->type == CI_U8) {
        long x = (long)*(uint8_t *)it->ptr + (long)(dir * place);
        if (x < (long)it->vmin) x = (long)it->vmin;
        if (x > (long)it->vmax) x = (long)it->vmax;
        *(uint8_t *)it->ptr = (uint8_t)x;
        if (it->ptr == (void *)&cfg.codec.hp_vol_l) {   /* hp_vol 同步左右声道并立即生效 */
            cfg.codec.hp_vol_r = (uint8_t)x;
            WM8978_HPvol_Set(cfg.codec.hp_vol_l, cfg.codec.hp_vol_r);
        }
    } else if (it->type == CI_U32) {
        long x = (long)*(uint32_t *)it->ptr + (long)(dir * place);
        if (x < (long)it->vmin) x = (long)it->vmin;
        if (x > (long)it->vmax) x = (long)it->vmax;
        *(uint32_t *)it->ptr = (uint32_t)x;
    } else if (it->type == CI_FLOAT) {
        double x = (double)*(float *)it->ptr + dir * place;
        if (x < (double)it->vmin) x = it->vmin;
        if (x > (double)it->vmax) x = it->vmax;
        *(float *)it->ptr = (float)x;
    }
}

static void ci_key(key_id_t k)
{
    const ci_item_t *it = &s_ci[s_ci_sel];

    if (!s_ci_edit) {
        /* 选择状态: 上下选字段, 中键进入修改 */
        switch (k) {
        case KEY_ID_UP:   if (s_ci_sel > 0) s_ci_sel--; break;
        case KEY_ID_DOWN: if (s_ci_sel < CI_N - 1) s_ci_sel++; break;
        case KEY_ID_MID:  s_ci_edit = true; s_ci_cursor = 0; break;
        default: break;
        }
    } else {
        /* 修改状态: 左右选修改位(数值跳过小数点), 上下按该位权值加减/换字符, 中键退出 */
        char v[24];
        int vlen = 1;
        bool numeric = (it->type == CI_INT || it->type == CI_U8 ||
                        it->type == CI_U32 || it->type == CI_FLOAT);
        if (it->type == CI_STR || numeric) {
            vlen = ci_value_str(it, v, sizeof(v));
            if (vlen < 1) vlen = 1;
        }
        switch (k) {
        case KEY_ID_LEFT: {
            int c = s_ci_cursor;
            do { if (c > 0) c--; } while (numeric && c > 0 && v[c] == '.');
            s_ci_cursor = c;
            break;
        }
        case KEY_ID_RIGHT: {
            int c = s_ci_cursor;
            do { if (c < vlen - 1) c++; } while (numeric && c < vlen - 1 && v[c] == '.');
            s_ci_cursor = c;
            break;
        }
        case KEY_ID_UP:    ci_edit_step(it, +1); break;
        case KEY_ID_DOWN:  ci_edit_step(it, -1); break;
        case KEY_ID_MID:   s_ci_edit = false; break;
        default: break;
        }
        if (s_ci_cursor >= vlen) s_ci_cursor = vlen - 1;
        if (s_ci_cursor < 0) s_ci_cursor = 0;
        if (numeric && s_ci_cursor < vlen && v[s_ci_cursor] == '.') s_ci_cursor = (vlen > 1) ? vlen - 1 : 0;
    }

    if (s_ci_sel < s_ci_scroll) s_ci_scroll = s_ci_sel;
    if (s_ci_sel >= s_ci_scroll + CI_ROWS) s_ci_scroll = s_ci_sel - (CI_ROWS - 1);
    s_cfg_dirty = true;
}

static void draw_page_cfg_set(void)
{
    lcd_line(0, CYAN, 12, "CFG SET %d/%d%s", s_ci_sel + 1, CI_N, s_ci_edit ? " EDIT" : "");

    for (int r = 0; r < CI_ROWS; r++) {
        int i = s_ci_scroll + r;
        if (i >= CI_N) break;
        const ci_item_t *it = &s_ci[i];
        int y = 12 + r * 12;

        if (i != s_ci_sel) {
            switch (it->type) {
            case CI_BOOL:  lcd_line(y, WHITE, 12, " %s %s", it->name, *(bool *)it->ptr ? "Y" : "N"); break;
            case CI_INT:   lcd_line(y, WHITE, 12, " %s %d", it->name, *(int *)it->ptr); break;
            case CI_U8:    lcd_line(y, WHITE, 12, " %s %u", it->name, (unsigned)*(uint8_t *)it->ptr); break;
            case CI_U32:   lcd_line(y, WHITE, 12, " %s %lu", it->name, (unsigned long)*(uint32_t *)it->ptr); break;
            case CI_FLOAT: lcd_line(y, WHITE, 12, " %s %.*f", it->name, it->dec, (double)*(float *)it->ptr); break;
            case CI_STR:   lcd_line(y, WHITE, 12, " %s %s", it->name, (char *)it->ptr); break;
            case CI_ENUM: {
                int v = *(int *)it->ptr;
                if (v < 0 || v >= it->nopts) v = 0;
                lcd_line(y, WHITE, 12, " %s %s", it->name, it->opts[v]);
                break;
            }
            default: break;
            }
            continue;
        }

        /* 选中行: 手工绘制; 修改状态反色高亮光标位 */
        char val[24];
        int vlen = ci_value_str(it, val, sizeof(val));
        int pos = s_ci_cursor;
        if (pos < 0) pos = 0;
        if (vlen > 0 && pos >= vlen) pos = vlen - 1;
        if (vlen == 0) pos = 0;

        char head[16];
        int hl = 0;
        head[hl++] = '>';
        for (const char *s = it->name; *s && hl < 13; ) head[hl++] = *s++;
        head[hl++] = ' ';
        head[hl] = '\0';

        LCD_Fill(0, (uint16_t)y, LCD_W, (uint16_t)(y + 12), BLACK);
        LCD_ShowString(0, (uint16_t)y, (const uint8_t *)head, YELLOW, BLACK, 12, 0);
        LCD_ShowString((uint16_t)(hl * 6), (uint16_t)y, (const uint8_t *)val, YELLOW, BLACK, 12, 0);

        int cx = hl + pos;
        if (s_ci_edit && vlen > 0 && cx < 21)
            LCD_ShowChar((uint16_t)(cx * 6), (uint16_t)y, (uint8_t)val[pos], BLACK, YELLOW, 12, 0);
    }
}

/* ================= LCD 主任务 ================= */
#define LCD_PAGE_NUM    5                  /* 页数: 0主页 1解码 2GPS 3日志 4配置 */
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
        if (page == 0)      draw_page_main();       //FT8CN 风格主页(状态+瀑布+发射)
        else if (page == 1) draw_page_decode();     //解码列表(文本优先, 配色)
        else if (page == 2) draw_page_gps(&g);      //GPS 概览(信息+卫星+信号合并)
        else if (page == 3) draw_page_log();        //日志(呼号/网格/信号/频率)
        else if (page == 4) draw_page_cfg_set();    //配置设置, 可设置 cfg 所有内容

        LCD_Flush();                       /* 画完一整帧后一次性推送 */

        /* 配置改动后约 2s 落盘到 /storage/cfg.txt */
        {
            static uint32_t save_tick = 0;
            if (s_cfg_dirty && (++save_tick % 100) == 0) {
                if (cfg_store_save(&cfg) == ESP_OK) s_cfg_dirty = false;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/* ================= RGB LED 呼吸灯 ================= */
static void rgb_led_task_0(void *arg)
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




/* ================= RGB LED 状态灯 ================= */
static void rgb_led_task(void *arg)
{
    static uint8_t res = 0;
    if (res == 0)
    {
        led_init();
        ESP_LOGI(TAG, "RGB LED 状态灯任务启动");
        res = 1;
    }

    bool on = false;
    for (;;)
    {
        /* 发射时隙=红, 接收时隙=绿; 每 20ms 翻转闪动 */
        bool tx_slot = ft8_app_in_tx_slot();
        if (on) led_set_rgb(tx_slot ? 64 : 0, tx_slot ? 0 : 64, 0);
        else    led_set_rgb(0, 0, 0);
        on = !on;
        vTaskDelay(pdMS_TO_TICKS(500));
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
    if (event == KEY_EVENT_LONG_PRESS) {
        /* 方向键长按 = 连续变化(驱动每 ~120ms 重发); SET/RST/MID 长按不处理 */
        if (key_id == KEY_ID_SET || key_id == KEY_ID_RST || key_id == KEY_ID_MID) return;
    } else if (event != KEY_EVENT_CLICK) {
        return;
    }

    int page = s_lcd_page;

    /* SET=上一页, RST=下一页 */
    if (key_id == KEY_ID_SET) {
        s_lcd_page = (page + LCD_PAGE_NUM - 1) % LCD_PAGE_NUM;
        ESP_LOGI("APP", "SET 上一页 -> %d", s_lcd_page);
        return;
    }
    if (key_id == KEY_ID_RST) {
        s_lcd_page = (page + 1) % LCD_PAGE_NUM;
        ESP_LOGI("APP", "RST 下一页 -> %d", s_lcd_page);
        return;
    }

    if (page == 0) {                       /* 主页: 上下选设置项, 左右调整(长按连续) */
        if (key_id == KEY_ID_UP)        { if (s_main_sel > 0) s_main_sel--; }
        else if (key_id == KEY_ID_DOWN) { if (s_main_sel < MAIN_ITEM_N - 1) s_main_sel++; }
        else if (key_id == KEY_ID_LEFT)  main_adjust(-1);
        else if (key_id == KEY_ID_RIGHT) main_adjust(+1);
        return;
    }

    if (page == 1) {                       /* 解码页: 上下选, 左键设目标, 长按右键清空 */
        int cnt = decode_count();
        if (key_id == KEY_ID_UP)        { if (s_rx_sel > 0) s_rx_sel--; }        /* 上=更旧 */
        else if (key_id == KEY_ID_DOWN) { if (s_rx_sel < cnt - 1) s_rx_sel++; }  /* 下=更新 */
        else if (key_id == KEY_ID_LEFT) {
            const ft8_rx_msg_t *m = decode_at(s_rx_sel, cnt);
            if (m && m->call_de[0]) {
                strncpy(cfg.tx.call_to, m->call_de, sizeof(cfg.tx.call_to) - 1);
                cfg.tx.call_to[sizeof(cfg.tx.call_to) - 1] = '\0';
                strncpy(cfg.qso.target_callsign, m->call_de, sizeof(cfg.qso.target_callsign) - 1);
                cfg.qso.target_callsign[sizeof(cfg.qso.target_callsign) - 1] = '\0';
                cfg.tx.type = FT8_APP_MSG_CALL;
                ESP_LOGI("APP", "目标呼号设为 %s", m->call_de);
            }
        }
        else if (key_id == KEY_ID_RIGHT && event == KEY_EVENT_LONG_PRESS) {
            ft8_rx_log_clear();            /* 长按右键清空列表 */
            s_rx_sel = 0;
            s_rx_last_cnt = 0;
            ESP_LOGI("APP", "清空解码列表");
        }
        return;
    }

    if (page == 3) {                       /* 日志页: 上下选行, 中键详情, 左右横滚 */
        if (s_log_detail) {
            if (key_id == KEY_ID_MID)        s_log_detail = 0;
            else if (key_id == KEY_ID_LEFT)  { if (s_log_hscroll > 0) s_log_hscroll--; }
            else if (key_id == KEY_ID_RIGHT) s_log_hscroll++;
            return;
        }
        if (key_id == KEY_ID_UP)         { if (s_log_sel > 0) s_log_sel--; }
        else if (key_id == KEY_ID_DOWN)  s_log_sel++;
        else if (key_id == KEY_ID_MID)   { s_log_detail = 1; s_log_hscroll = 0; }
        return;
    }

    if (page == 4) {                       /* 配置页 */
        ci_key(key_id);
        return;
    }

    /* 其它页(主页/GPS): 方向键不用于翻页; 中键回主页。翻页只用 SET/RST。 */
    if (key_id == KEY_ID_MID) s_lcd_page = 0;
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
    snprintf(cfg.band,     sizeof(cfg.band),     "40m");    // QSO 日志用频段(设备无射频信息, 手动指定)
    cfg.qso_freq_mhz        = 7.074000f;                    // QSO 日志用频率 MHz
    cfg.tx.type             = FT8_APP_MSG_CQ;               /* 第几类消息: CQ / CALL / REPORT / R_REPORT / RRR / RR73 / 73 */
    cfg.tx.cq_modifier[0]   = '\0';                         /* "DX"/"WW"/"TEST"... 仅 CQ 类用 */
    cfg.tx.call_to[0]       = '\0';                         /* 目标呼号(类型 2~6 用)，如 "BG5ABC" */
    cfg.tx.rst_db           = -12;                          /* 信号报告 dB(类型 3/4 用)：REPORT 发 -12，R_REPORT 发 R-12 */
    cfg.audio_freq_hz       = 1500.0f;                      /* 音频中心(8-GFSK tone0) */
    cfg.audio_level         = 0.80f;                        /* 发射电平 */ /*!< 发射幅度 0~1，防削波建议 ≤0.9 */
    cfg.rx_f_max            = 3000.0f;                      /* 解码频率上限 */
    cfg.rx_f_min            = 50.0f;                        /* 解码频率下限 */
    cfg.max_candidates      = 50;                           /*每时隙解码耗时 ≈ 候选数(max_candidates) × 每个候选迭代数(ldpc_iterations) × 单次迭代成本*/
    cfg.ldpc_iterations     = 1;

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

    cfg.usb_mount_enable    = false;       // false=不启动 USB, 只本地写 /storage/log.txt    是否把日志分区作为 U 盘挂载(USB MSC 暴露给 PC);  只本地挂载 FAT 写日志, 不启动 USB 
    
    //true 
    //false

    //启动任务============================================================================================
    /* QSO 日志: 注册回调并初始化 FAT 分区; 先读 cfg.txt 再决定是否启动 U 盘 */
    ft8_app_set_qso_callback(qso_log_on_qso, NULL);
    if (qso_log_init() == ESP_OK) {
        cfg_store_load(&cfg);                  /* 覆盖已持久化字段(含 usb_mount_enable) */
        if (cfg.usb_mount_enable)
            qso_log_usb_start();               /* 按 cfg.txt 的值启动 USB 大容量存储 */
    } else {
        ESP_LOGW(TAG, "QSO 日志存储初始化失败(不影响收发)");
    }

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