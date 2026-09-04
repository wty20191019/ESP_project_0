/* 多任务演示：回声测试 + RGB LED
 *  - wm8978_echo_task：WM8978 数字回声（ADC 采集 -> 回声效果 -> DAC 回放）
 *  - rgb_led_task：LED(WS2812) 三色呼吸循环
 *
 * 接线约定（见 wm8978_i2c.h / wm8978_i2s.h）：
 *   I2C0: SCL=GPIO8, SDA=GPIO9
 *   I2S0: MCLK=GPIO12, BCLK=GPIO13, LRCK=GPIO14, DOUT=GPIO15, DIN=GPIO16
 *   LED : GPIO48 (见 led.h)
 */
#include <stdint.h>
#include <stddef.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "wm8978.h"
#include "wm8978_i2s.h"
#include "led.h"

#define TAG "main"

/* ====== 回声/采集参数 ====== */
#define SAMPLE_RATE     16000           /* 采样率(Hz) */
#define ECHO_DELAY_MS   2               /* 回声延迟，200ms 听感清晰 */
#define ECHO_FEEDBACK   0.55f           /* 回声反馈系数(0~1)，越大回音越长 */
#define INPUT_GAIN      0.85f           /* 输入信号幅度(避免反馈叠加削波) */

/* 输入源选择：
 *   0 = 线路输入 LINE IN（R2/L2）
 *   1 = 咪头输入 MIC（RIP/LIP，走 INPPGA）
 *   2 = 两路同时输入
 */
#define ECHO_INPUT_SRC  2

#define ECHO_DELAY_FRAMES   (SAMPLE_RATE * ECHO_DELAY_MS / 1000) /* 延迟环帧数 */
#define BLOCK_FRAMES        320         /* 每块帧数(20ms)，一帧=左+右 int16 */

static int16_t s_audio_buf[BLOCK_FRAMES * 2];   /* 收发共用缓冲(交织) */

/* 左/右声道各自的回声延迟环(浮点，含反馈回路) */
static float s_ring_l[ECHO_DELAY_FRAMES];
static float s_ring_r[ECHO_DELAY_FRAMES];
static int s_ring_pos = 0;

/* 16bit 限幅 */
static inline int16_t clamp_s16(float v)
{
    if (v > 32767.0f) return 32767;
    if (v < -32768.0f) return -32768;
    return (int16_t)v;
}

/* 回声处理：y[n] = x[n] + d[n]；d[n] = x[n] + FB*d[n-D]，逐帧原地处理 */
static void echo_process(int16_t *buf, int nframes)
{
    for (int i = 0; i < nframes; i++) {
        float xl = (float)buf[i * 2 + 0] * INPUT_GAIN;
        float xr = (float)buf[i * 2 + 1] * INPUT_GAIN;

        float dl = s_ring_l[s_ring_pos];        /* D 帧前的信号 */
        float dr = s_ring_r[s_ring_pos];

        s_ring_l[s_ring_pos] = xl + dl * ECHO_FEEDBACK;     /* 更新延迟环(回音反馈) */
        s_ring_r[s_ring_pos] = xr + dr * ECHO_FEEDBACK;
        if (++s_ring_pos >= ECHO_DELAY_FRAMES) s_ring_pos = 0;

        buf[i * 2 + 0] = clamp_s16(xl + dl);    /* 输出=原声+回音 */
        buf[i * 2 + 1] = clamp_s16(xr + dr);
    }
}

/* ================= 任务1：WM8978 回声测试 ================= */
static void wm8978_echo_task(void *arg)
{
    const char *T = "echo";
    ESP_LOGI(T, "回声任务启动(SR=%dHz, 延迟=%dms, 输入源=%d)",
             SAMPLE_RATE, ECHO_DELAY_MS, ECHO_INPUT_SRC);

    /* 上电稳定窗口，避免启动瞬间 I2C 偶发失败 */
    vTaskDelay(pdMS_TO_TICKS(200));

    if (WM8978_Init() != 0) {
        ESP_LOGE(T, "WM8978 初始化失败，请检查 I2C 接线/供电");
        goto fail;
    }

    /* 打开 ADC(采集输入)与 DAC(回声输出) */
    WM8978_ADDA_Cfg(1, 1);

    /* 按输入源配置输入通路：MIC 对应 RIP/LIP(PGA)，LineIn 对应 R2/L2 */
    if (ECHO_INPUT_SRC == 1) {
        WM8978_Input_Cfg(1, 0, 0);              /* 仅咪头 RIP/LIP */
        WM8978_MIC_Gain(40);                    /* MIC PGA 增益 */
        ESP_LOGI(T, "输入源: MIC (RIP/LIP)");
    } else if (ECHO_INPUT_SRC == 0) {
        WM8978_Input_Cfg(0, 1, 0);              /* 仅线路输入 R2/L2 */
        ESP_LOGI(T, "输入源: LINE IN (R2/L2)");
    } else {
        WM8978_Input_Cfg(1, 1, 0);              /* 两路同时使能 */
        WM8978_MIC_Gain(40);
        ESP_LOGI(T, "输入源: MIC + LINE IN 同时");
    }

    WM8978_Output_Cfg(1, 0);        /* DAC 输出使能，关闭模拟 Bypass */
    WM8978_I2S_Cfg(2, 0);           /* 飞利浦标准 I2S，16 位数据长度 */
    WM8978_HPvol_Set(50, 50);       /* 耳机音量 */
    WM8978_SPKvol_Set(40);          /* 喇叭音量 */

    /* 初始化 I2S0 全双工并同时启动 TX/RX */
    esp_err_t err = wm8978_i2s_init(SAMPLE_RATE);
    if (err != ESP_OK) {
        ESP_LOGE(T, "I2S 初始化失败: %s", esp_err_to_name(err));
        goto fail;
    }
    err = wm8978_i2s_start_rx();
    err |= wm8978_i2s_start_tx();
    if (err != ESP_OK) {
        ESP_LOGE(T, "I2S 启动失败: %s", esp_err_to_name(err));
        goto fail;
    }

    ESP_LOGI(T, "数字回声已开启，请对输入源发声...");

    /* 实时回环：读 ADC -> 回声处理 -> 写 DAC */
    for (;;) {
        size_t got = 0;
        err = wm8978_i2s_read(s_audio_buf, sizeof(s_audio_buf), &got, 1000);
        if (err != ESP_OK || got == 0) {
            ESP_LOGW(T, "I2S 读失败: %s", esp_err_to_name(err));
            continue;
        }
        echo_process(s_audio_buf, (int)(got / 4));   /* got/4 = 帧数 */

        size_t sent = 0;
        err = wm8978_i2s_write(s_audio_buf, got, &sent, 1000);
        if (err != ESP_OK) {
            ESP_LOGW(T, "I2S 写失败: %s", esp_err_to_name(err));
        }
    }

fail:
    ESP_LOGE(T, "回声任务异常退出");
    vTaskDelete(NULL);
}

/* ================= 任务2：RGB LED 呼吸 ================= */
static void rgb_led_task(void *arg)
{
    const char *T = "rgb_led";
    led_init();
    ESP_LOGI(T, "RGB LED 任务启动");

    for (;;) {
        for (uint8_t i = 0; i <= 250; i += 5) { led_set_rgb(i, 0, 0); vTaskDelay(pdMS_TO_TICKS(6)); }
        for (uint8_t i = 250; i > 0; i -= 5)  { led_set_rgb(i, 0, 0); vTaskDelay(pdMS_TO_TICKS(6)); }

        for (uint8_t i = 0; i <= 250; i += 5) { led_set_rgb(0, i, 0); vTaskDelay(pdMS_TO_TICKS(6)); }
        for (uint8_t i = 250; i > 0; i -= 5)  { led_set_rgb(0, i, 0); vTaskDelay(pdMS_TO_TICKS(6)); }

        for (uint8_t i = 0; i <= 250; i += 5) { led_set_rgb(0, 0, i); vTaskDelay(pdMS_TO_TICKS(6)); }
        for (uint8_t i = 250; i > 0; i -= 5)  { led_set_rgb(0, 0, i); vTaskDelay(pdMS_TO_TICKS(6)); }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "创建任务: wm8978_echo_task / rgb_led_task");

    xTaskCreatePinnedToCore(wm8978_echo_task, "wm8978_echo", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(rgb_led_task, "rgb_led", 2048, NULL, 5, NULL, 0);
}
