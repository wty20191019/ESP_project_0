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

/* ================= LCD 主任务 ================= */
#define LCD_PAGE_NUM    3                  /* 页数: 0=信息 1=卫星 2=信号 */
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
        else                draw_page_signal(g);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* RGB LED 呼吸灯*/
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
        for (int i = 0; i <= 255; i += 5) { led_set_rgb(i, 0, 0); vTaskDelay(pdMS_TO_TICKS(10)); }
        for (int i = 255; i > 0; i -= 5)  { led_set_rgb(i, 0, 0); vTaskDelay(pdMS_TO_TICKS(10)); }

        for (int i = 0; i <= 255; i += 5) { led_set_rgb(0, i, 0); vTaskDelay(pdMS_TO_TICKS(10)); }
        for (int i = 255; i > 0; i -= 5)  { led_set_rgb(0, i, 0); vTaskDelay(pdMS_TO_TICKS(10)); }

        for (int i = 0; i <= 255; i += 5) { led_set_rgb(0, 0, i); vTaskDelay(pdMS_TO_TICKS(10)); }
        for (int i = 255; i > 0; i -= 5)  { led_set_rgb(0, 0, i); vTaskDelay(pdMS_TO_TICKS(10)); }
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

    /* ====== FT8/FT4 配置示例 ====== */
    ft8_app_config_t cfg;
    ft8_app_config_default(&cfg);

    cfg.protocol      = FTX_PROTOCOL_FT8;    /* FTX_PROTOCOL_FT4 切到 FT4(7.5s 时隙) */
    cfg.tx_enable     = true;                /* 参与发射(仅在选中奇偶时隙) */
    cfg.rx_enable     = true;                /* 持续解码 */
    cfg.utc_enable    = true;                /* 时隙对齐 UTC(:00/:15/:30/:45)，需先 SNTP 校时 */
    cfg.tx_slot_parity = 0;                  /* 0=偶时隙发 / 1=奇时隙发，自动与对端交替 */
    cfg.tx_delay_ms   = 500;                 /* 本台时隙内再延时发射 */
    snprintf(cfg.callsign, sizeof(cfg.callsign), "BG7ABC");
    snprintf(cfg.grid,     sizeof(cfg.grid),     "JO70");
    cfg.msg_mode      = FT8_APP_MSG_CQ;      /* CQ 呼叫，或 FT8_APP_MSG_CALL 呼叫指定台 */
    cfg.cq_modifier[0] = '\0';               /* "DX"/"WW"/"TEST"... */
    cfg.audio_freq_hz = 1500.0f;             /* 音频中心(8-GFSK tone0) */
    cfg.audio_level   = 0.10f;               /* 发射电平 */
    cfg.rx_f_max      = 3000.0f;             /* 解码频率上限 */

    if (ft8_app_start(&cfg) != ESP_OK) {
        ESP_LOGE(TAG, "ft8_app 启动失败");
        return;
    }

    xTaskCreatePinnedToCore(LCD_task, "LCD", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(rgb_led_task, "rgb_led", 2048, NULL, 1, NULL, 0);
}