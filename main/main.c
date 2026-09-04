/* 1kHz 正弦波输出测试：
 * WM8978(ESP32-S3 I2S0 + I2C0) 初始化后，由主循环实时生成 1kHz 双声道
 * 正弦波并送入 I2S0 播放，耳机/喇叭应能听到 1kHz 单音。
 *
 * 接线约定（见 wm8978_i2c.h / wm8978_i2s.h）：
 *   I2C0: SCL=GPIO8, SDA=GPIO9
 *   I2S0: MCLK=GPIO12, BCLK=GPIO13, LRCK=GPIO14, DOUT=GPIO15, DIN=GPIO16
 */
#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "wm8978.h"
#include "wm8978_i2s.h"

#define TAG "tone1000"

#define SAMPLE_RATE     44100           /* 采样率(Hz) */
#define TONE_FREQ       1000.0f         /* 测试音频率(Hz) */
#define TONE_AMP        0.4f            /* 幅度(0~1，避免削波) */
#define FRAME_NUM       512             /* 每块帧数(一帧 = 左+右 两个 16bit 采样) */

/* 相位累加(32bit)方式产生正弦，避免非整数周期导致频率漂移 */
#define PHASE_STEP      ((uint32_t)(4294967296.0 * (TONE_FREQ / SAMPLE_RATE)))
#define PHASE_TO_RAD    (6.28318530718f / 4294967296.0f)

static int16_t s_audio_buf[FRAME_NUM * 2];   /* 左右声道交织缓冲 */

static void tone_fill(int16_t *buf, size_t nframes)
{
    static uint32_t phase = 0;          /* 相位累加器 */
    for (size_t i = 0; i < nframes; i++) {
        phase += PHASE_STEP;
        float rad = (float)phase * PHASE_TO_RAD;
        int16_t v = (int16_t)(sinf(rad) * 32767.0f * TONE_AMP);
        buf[i * 2 + 0] = v;             /* 左声道 */
        buf[i * 2 + 1] = v;             /* 右声道 */
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "WM8978 1kHz 声音输出测试开始...");

    /* 上电后等待电源/CODEC 内部复位稳定，避免启动瞬间首帧 I2C 偶发失败 */
    vTaskDelay(pdMS_TO_TICKS(200));

    /* 1. WM8978 初始化(内含 I2C0 初始化与寄存器默认配置) */
    if (WM8978_Init() != 0) {
        ESP_LOGE(TAG, "WM8978 初始化失败，请检查 I2C 接线/供电");
        return;
    }

    /* 2. 配置音频通路：只走 DAC 播放，关闭录音输入与 Bypass */
    WM8978_ADDA_Cfg(1, 0);          /* 开启 DAC，关闭 ADC */
    WM8978_Input_Cfg(0, 0, 0);      /* 关闭 MIC/LineIn/AUX 输入 */
    WM8978_Output_Cfg(1, 0);        /* 开启 DAC 输出，关闭 Bypass */
    WM8978_I2S_Cfg(2, 0);           /* 飞利浦标准 I2S，16 位数据长度(与 I2S 驱动一致) */
    WM8978_HPvol_Set(50, 50);       /* 耳机音量 */
    WM8978_SPKvol_Set(40);          /* 喇叭音量 */

    /* 3. I2S0 初始化并启动 TX(播放)通道，采样率 44.1kHz */
    esp_err_t err = wm8978_i2s_init(SAMPLE_RATE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2S 初始化失败: %s", esp_err_to_name(err));
        return;
    }
    err = wm8978_i2s_start_tx();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2S 启动失败: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "开始输出 1kHz 正弦波(SR=%dHz, 幅度=%d%%)...",
             SAMPLE_RATE, (int)(TONE_AMP * 100));

    /* 4. 持续生成并写入音频数据(i2s 写满 DMA 环形缓冲后会自动阻塞，
     *    因此输出节奏与采样率自然同步) */
    while (1) {
        tone_fill(s_audio_buf, FRAME_NUM);
        size_t written = 0;
        err = wm8978_i2s_write(s_audio_buf, sizeof(s_audio_buf), &written, 1000);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "I2S 写失败: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}
