/* 入口：FT8/FT4 应用(模块 ft8_app) + 可选 RGB LED 呼吸
 *
 * 说明：
 *  - ft8_app 为单任务控制器：持续解码，并在每个协议槽按配置定时发射。
 *  - 所有可调项见 main/ft8_app.h 的 ft8_app_config_t。
 *
 * 接线约定（见 wm8978_i2c.h / wm8978_i2s.h / led.h）：
 *   I2C0: SCL=GPIO8, SDA=GPIO9
 *   I2S0: MCLK=GPIO12, BCLK=GPIO13, LRCK=GPIO14, DOUT=GPIO15, DIN=GPIO16
 *   LED : GPIO48
 */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "ft8_app.h"
#include "led.h"

#define TAG "main"

/* RGB LED 呼吸(可选)；不要可整段删除 */
static void rgb_led_task(void *arg)
{
    led_init();
    for (;;) {
        for (uint8_t i = 0; i <= 200; i += 5) { led_set_rgb(i, 0, 0); vTaskDelay(pdMS_TO_TICKS(10)); }
        for (uint8_t i = 200; i > 0; i -= 5)  { led_set_rgb(i, 0, 0); vTaskDelay(pdMS_TO_TICKS(10)); }
        for (uint8_t i = 0; i <= 200; i += 5) { led_set_rgb(0, 0, i); vTaskDelay(pdMS_TO_TICKS(10)); }
        for (uint8_t i = 200; i > 0; i -= 5)  { led_set_rgb(0, 0, i); vTaskDelay(pdMS_TO_TICKS(10)); }
    }
}

void app_main(void)
{
    /* ====== FT8/FT4 配置示例 ====== */
    ft8_app_config_t cfg;
    ft8_app_config_default(&cfg);

    cfg.protocol      = FTX_PROTOCOL_FT8;    /* FTX_PROTOCOL_FT4 切到 FT4(7.5s 槽) */
    cfg.tx_enable     = true;                /* 允许发射 */
    cfg.rx_enable     = true;                /* 持续解码 */
    snprintf(cfg.callsign, sizeof(cfg.callsign), "BG7ABC");
    snprintf(cfg.grid,     sizeof(cfg.grid),     "JO70");
    cfg.msg_mode      = FT8_APP_MSG_CQ;      /* CQ 呼叫，或 FT8_APP_MSG_CALL 呼叫指定台 */
    cfg.cq_modifier[0] = '\0';               /* "DX"/"WW"/"TEST"... */
    cfg.tx_delay_ms   = 500;                 /* 槽开始后延时发射 */
    cfg.time_offset_ms = 0;                  /* 时间栅格偏移(ms)，接真实电台时对齐 UTC */
    cfg.audio_freq_hz = 1200.0f;             /* 音频中心频率 */
    cfg.audio_level   = 0.10f;               /* 发射音量 */
    cfg.rx_f_min      = 0.0f;
    cfg.rx_f_max      = 4000.0f;

    if (ft8_app_start(&cfg) != ESP_OK) {
        ESP_LOGE(TAG, "ft8_app 启动失败");
        return;
    }

    xTaskCreatePinnedToCore(rgb_led_task, "rgb_led", 2048, NULL, 1, NULL, 0);
}
