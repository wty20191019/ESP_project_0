#include "ft8_app.h"
#include <string.h>
#include <math.h>
#include <stdio.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ft8/encode.h"
#include "ft8/decode.h"
#include "ft8/message.h"
#include "common/monitor.h"
#include "wm8978.h"
#include "wm8978_i2s.h"

/* ============================================================
 * FT8/FT4 应用模块（单任务实现）
 *
 * 单任务职责：
 *   1. 每个音频符号周期(FT8 0.16s / FT4 0.048s)：
 *      - 从 I2S0 RX(WM8978 ADC) 读一个符号帧喂给 monitor 瀑布；
 *      - 按“时间偏移+发射延时”判断当前是否处于发射窗，是则把对应
 *        tone 的 8FSK/4FSK 音频写入 I2S0 TX(WM8978 DAC)，否则补零。
 *   2. 攒满一个协议槽(FT8 15s / FT4 7.5s)后执行一次解码并打印，然后清空重来。
 *
 * 音频采样率固定 12kHz（ft8_lib 的 monitor/调制均按此设计）。
 * ============================================================ */

#define FT8_AUDIO_RATE      12000
#define MAX_SYMBOL_SAMPLES  1920    /* FT8 每符号样本数(FT4 为 576) */
#define STACK_SIZE          32768

static const char *TAG = "ft8_app";

static ft8_app_config_t s_cfg;      /* 配置副本 */
static TaskHandle_t s_task = NULL;  /* 任务句柄 */

/* ---------------- 协议参数换算 ---------------- */
static bool app_is_ft4(void)   { return s_cfg.protocol == FTX_PROTOCOL_FT4; }
static int  app_sym_samples(void)  { return app_is_ft4() ? 576 : 1920; }
static int  app_nsym(void)         { return app_is_ft4() ? FT4_NN : FT8_NN; }
static int64_t app_slot_us(void)   { return app_is_ft4() ? (int64_t)7500000 : (int64_t)15000000; }
static int64_t app_sym_us(void)    { return app_is_ft4() ? 48000 : 160000; }
static float app_tone_spacing(void){ return FT8_AUDIO_RATE / (float)app_sym_samples(); }

/* ============================================================
 * 发送消息组装：文本 -> payload -> tone 序列
 * ============================================================ */
static esp_err_t tx_encode_message(uint8_t *tones)
{
    ftx_message_t msg;
    char text[96];
    ftx_message_rc_t rc;

    /* 逐条尝试：CALL 目标模式 -> 带修饰的 CQ -> 纯 CQ 兜底 */
    ftx_message_init(&msg);

    if (s_cfg.msg_mode == FT8_APP_MSG_CALL && s_cfg.call_to[0]) {
        snprintf(text, sizeof(text), "%s %s %s", s_cfg.call_to, s_cfg.callsign, s_cfg.grid);
        rc = ftx_message_encode(&msg, NULL, text);
        if (rc == FTX_MESSAGE_RC_OK) goto ok;
    }
    if (s_cfg.cq_modifier[0]) {
        snprintf(text, sizeof(text), "CQ %s %s %s", s_cfg.cq_modifier, s_cfg.callsign, s_cfg.grid);
        rc = ftx_message_encode(&msg, NULL, text);
        if (rc == FTX_MESSAGE_RC_OK) goto ok;
        ESP_LOGW(TAG, "FT8 无法表达 CQ %s，回退为纯 CQ", s_cfg.cq_modifier);
    }
    snprintf(text, sizeof(text), "CQ %s %s", s_cfg.callsign, s_cfg.grid);
    rc = ftx_message_encode(&msg, NULL, text);
    if (rc != FTX_MESSAGE_RC_OK) {
        ESP_LOGE(TAG, "消息编码失败: %s", text);
        return ESP_FAIL;
    }
ok:
    if (app_is_ft4()) ft4_encode(msg.payload, tones);   /* FT4：105 符号 4FSK */
    else              ft8_encode(msg.payload, tones);   /* FT8：79 符号 8FSK */
    ESP_LOGI(TAG, "发射消息: %s", text);
    return ESP_OK;
}

/* ---------------- 消息解码文本还原回调(标准呼号即可，哈希查询为空) ---------------- */
static bool hash_lookup(ftx_callsign_hash_type_t type, uint32_t hash, char *callsign)
{
    (void)type; (void)hash; (void)callsign;
    return false;
}
static void hash_save(const char *callsign, uint32_t n22)
{
    (void)callsign; (void)n22;
}
static ftx_callsign_hash_interface_t s_hash_if = { hash_lookup, hash_save };

/* ============================================================
 * FT8/FT4 主任务
 * ============================================================ */
static void ft8_app_task(void *arg)
{
    (void)arg;
    const char *T = "ft8_app";

    /* --- 协议相关本地量 --- */
    const bool ft4 = app_is_ft4();
    const int sym_samples = app_sym_samples();   /* 每符号样本数(单声道帧) */
    const int nsym = app_nsym();                 /* 符号总数 */
    const int64_t slot_us = app_slot_us();       /* 槽时长 */
    const int64_t sym_us = app_sym_us();         /* 符号时长 */
    const float spacing = app_tone_spacing();    /* tone 间距 */

    static int16_t ablk[MAX_SYMBOL_SAMPLES * 2];    /* 收发共用立体声块 */
    static float   fr[MAX_SYMBOL_SAMPLES];          /* 单声道浮点帧 */
    static uint8_t tones[FT4_NN];                   /* 发射 tone 序列(取最大长度) */

    /* --- 编码发射消息 --- */
    if (s_cfg.tx_enable) {
        if (tx_encode_message(tones) != ESP_OK) {
            s_cfg.tx_enable = false;    /* 编码失败则关闭发射 */
        }
    }

    /* --- 建立解码瀑布(仅 RX 需要) --- */
    monitor_t mon;
    if (s_cfg.rx_enable) {
        monitor_config_t mc = {
            .f_min = s_cfg.rx_f_min,
            .f_max = s_cfg.rx_f_max,
            .sample_rate = FT8_AUDIO_RATE,
            .time_osr = (s_cfg.rx_time_osr > 0) ? s_cfg.rx_time_osr : 1,
            .freq_osr = (s_cfg.rx_freq_osr > 0) ? s_cfg.rx_freq_osr : 1,
            .protocol = s_cfg.protocol,
        };
        monitor_init(&mon, &mc);
        ESP_LOGI(T, "%s 解码启动: %d bins x %d blocks",
                 ft4 ? "FT4" : "FT8", mon.wf.num_bins, mon.wf.max_blocks);
    }

    /* --- TX 状态机 --- */
    bool tx_active = false;
    int tx_idx = 0;
    float phase = 0.0f;
    const int64_t tx_start_us = (int64_t)s_cfg.tx_delay_ms * 1000;
    const int64_t tx_dur_us = (int64_t)nsym * sym_us;
    const bool tx_fits = (tx_start_us + tx_dur_us) <= slot_us;

    if (s_cfg.tx_enable) {
        if (!tx_fits) {
            ESP_LOGW(T, "发射延时 %ums + 时长超槽，将跳过无法容纳的槽(请把延时调小)",
                     (unsigned)s_cfg.tx_delay_ms);
        }
    }

    ESP_LOGI(T, "%s 应用任务启动: 收=%d 发=%d 音频=%dHz/%0.0fHz 幅度=%0.2f",
             ft4 ? "FT4" : "FT8", s_cfg.rx_enable, s_cfg.tx_enable,
             FT8_AUDIO_RATE, (double)s_cfg.audio_freq_hz, (double)s_cfg.audio_level);

    /* ==================== 主循环 ==================== */
    for (;;) {
        /* 1. 读一个符号周期的音频(全双工 RX 始终启用，供解码并作为节拍) */
        int got = 0, bytes = sym_samples * 4;
        while (got < bytes) {
            size_t rd = 0;
            if (wm8978_i2s_read((uint8_t *)ablk + got, bytes - got, &rd, 1000) != ESP_OK || rd == 0) {
                ESP_LOGW(T, "I2S 读异常");
                vTaskDelay(pdMS_TO_TICKS(5));
                continue;
            }
            got += (int)rd;
        }

        /* 2. 喂解码瀑布(取左声道) */
        if (s_cfg.rx_enable) {
            for (int i = 0; i < sym_samples; i++) fr[i] = (float)ablk[i * 2] * (1.0f / 32768.0f);
            monitor_process(&mon, fr);
        }

        /* 3. 决定本符号发什么：发射窗内按序发 tone，否则补零 */
        int tx_tone = -1;
        if (s_cfg.tx_enable) {
            int64_t now = esp_timer_get_time() + (int64_t)s_cfg.time_offset_ms * 1000;
            int64_t t = now % slot_us;
            if (tx_active) {
                if (tx_idx < nsym) tx_tone = tones[tx_idx++];
                if (tx_idx >= nsym) { tx_active = false; ESP_LOGI(T, "[TX] 本槽发射完成"); }
            } else if (tx_fits && t >= tx_start_us && t < tx_start_us + tx_dur_us) {
                tx_active = true; tx_idx = 0;
                phase = 0.0f;
                tx_tone = tones[tx_idx++];
                ESP_LOGI(T, "[TX] 槽内 %d.%02ds 开始发射", (int)(t / 1000000), (int)((t % 1000000) / 10000));
            }
        }

        /* 4. 合成并写出本符号音频(发射 tone 或静音) */
        if (tx_tone >= 0) {
            float f = s_cfg.audio_freq_hz + tx_tone * spacing;
            float dph = 2.0f * (float)M_PI * f / FT8_AUDIO_RATE;
            for (int i = 0; i < sym_samples; i++) {
                phase += dph;
                if (phase > 2.0f * (float)M_PI) phase -= 2.0f * (float)M_PI;
                int16_t s = (int16_t)(sinf(phase) * 32767.0f * s_cfg.audio_level);
                ablk[i * 2 + 0] = s;
                ablk[i * 2 + 1] = s;
            }
        } else {
            memset(ablk, 0, (size_t)sym_samples * 4);
        }
        int sent = 0;
        while (sent < bytes) {
            size_t wr = 0;
            if (wm8978_i2s_write((const uint8_t *)ablk + sent, bytes - sent, &wr, 1000) != ESP_OK || wr == 0) {
                ESP_LOGW(T, "I2S 写异常");
                vTaskDelay(pdMS_TO_TICKS(5));
                continue;
            }
            sent += (int)wr;
        }

        /* 5. 攒满一个协议槽 -> 解码 */
        if (s_cfg.rx_enable && mon.wf.num_blocks >= mon.wf.max_blocks) {
            static ftx_candidate_t cands[128];
            int n = ftx_find_candidates(&mon.wf, s_cfg.max_candidates > 0 ? s_cfg.max_candidates : 60,
                                        cands, 0);
            for (int i = 0; i < n; i++) {
                ftx_message_t msg;
                ftx_decode_status_t st;
                if (!ftx_decode_candidate(&mon.wf, &cands[i],
                                          s_cfg.ldpc_iterations > 0 ? s_cfg.ldpc_iterations : 30,
                                          &msg, &st)) {
                    continue;
                }
                char text[64];
                ftx_message_offsets_t offs;
                if (ftx_message_decode(&msg, &s_hash_if, text, &offs) != FTX_MESSAGE_RC_OK) {
                    continue;
                }
                ESP_LOGI(T, "[RX] %s @%0.1fHz t=%0.2fs: %s",
                         ft4 ? "FT4" : "FT8", (double)st.freq, st.time, text);
                break;
            }
            monitor_reset(&mon);    /* 开始下一槽 */
        }
    }
}

/* ============================================================
 * 对外接口
 * ============================================================ */
void ft8_app_config_default(ft8_app_config_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));
    cfg->protocol        = FTX_PROTOCOL_FT8;
    cfg->tx_enable       = true;
    cfg->rx_enable       = true;
    snprintf(cfg->callsign, sizeof(cfg->callsign), "BG7ABC");
    snprintf(cfg->grid,     sizeof(cfg->grid),     "JO70");
    cfg->msg_mode         = FT8_APP_MSG_CQ;
    cfg->cq_modifier[0]   = 0;
    cfg->tx_delay_ms      = 500;       /* 槽开始后 0.5s 发射，默认消息可完整落入槽内 */
    cfg->time_offset_ms   = 0;         /* 0 = 以上电时刻为 15s/7.5s 栅格起点 */
    cfg->audio_freq_hz    = 1200.0f;
    cfg->audio_level      = 0.45f;
    cfg->rx_f_min         = 0.0f;
    cfg->rx_f_max         = 4000.0f;
    cfg->rx_time_osr      = 1;
    cfg->rx_freq_osr      = 1;
    cfg->max_candidates   = 60;
    cfg->ldpc_iterations  = 30;
}

esp_err_t ft8_app_start(const ft8_app_config_t *cfg)
{
    if (cfg == NULL) return ESP_ERR_INVALID_ARG;
    if (s_task != NULL) {
        ESP_LOGW(TAG, "FT8 应用已在运行，忽略本次启动");
        return ESP_OK;
    }
    s_cfg = *cfg;                       /* 拷贝配置 */

    /* 音频通路(W-M8978 + I2S0 全双工 12kHz)，模块内幂等初始化 */
    vTaskDelay(pdMS_TO_TICKS(200));     /* 上电稳定窗口，避免 I2C 偶发失败 */
    if (WM8978_Init() != 0) {
        ESP_LOGE(TAG, "WM8978 初始化失败");
        return ESP_FAIL;
    }
    WM8978_ADDA_Cfg(1, 1);              /* DAC + ADC */
    WM8978_Input_Cfg(1, 1, 0);          /* MIC + LINE IN */
    WM8978_MIC_Gain(40);
    WM8978_Output_Cfg(1, 0);
    WM8978_I2S_Cfg(2, 0);               /* 飞利浦, 16bit */
    WM8978_HPvol_Set(50, 50);
    WM8978_SPKvol_Set(40);
    ESP_ERROR_CHECK(wm8978_i2s_init(FT8_AUDIO_RATE));
    ESP_ERROR_CHECK(wm8978_i2s_start_rx());
    ESP_ERROR_CHECK(wm8978_i2s_start_tx());

    xTaskCreatePinnedToCore(ft8_app_task, "ft8_app", STACK_SIZE, NULL, 6, &s_task, 1);
    return ESP_OK;
}
