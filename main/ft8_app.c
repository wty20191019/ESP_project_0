#include "ft8_app.h"
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ft8/encode.h"
#include "ft8/decode.h"
#include "ft8/message.h"
#include "common/monitor.h"
#include "wm8978.h"
#include "wm8978_i2s.h"

/* ============================================================
 * FT8/FT4 应用模块 —— 收发拆分为两个独立任务，时间严格对齐
 *
 *  TX 任务(ft8_tx)：按奇偶时隙调度，用 esp_timer 忙等到目标时刻
 *                  才开始整窗写入(避免被解码/其它负载拖后)，起始
 *                  误差收敛到毫秒级。
 *  RX 任务(ft8_rx)：每拍读 ADC 喂瀑布，持续接收(不因发射暂停)，
 *                  每次时隙边界结束即整窗解析一次(结果去重)。
 *
 *  两个任务各自操作 I2S0 的 TX/RX 方向(全双工)，互不阻塞。
 *  波形：GFSK 高斯成形(官方 gen_ft8 算法)，PSRAM 预生成。
 *
 *  ⚠ 时隙栅格：默认以 esp_timer(上电时刻) 为 0 的本地栅格做严格
 *    对齐；若需真正对齐 UTC(:00/:15/:30/:45)，需先接入 SNTP 且提供
 *    毫秒级墙钟(当前 time() 只有秒级，无法把 esp_timer 相位映射到
 *    UTC 亚秒，只会保证奇偶相位正确)。启用 utc_enable 仅用于校验。
 * ============================================================ */

#define FT8_AUDIO_RATE      12000
#define MAX_SYMBOL_SAMPLES  1920
#define STACK_RX            (48 * 1024)   /* monitor FFT 需大栈 */
#define STACK_TX            (8 * 1024)

#define SNTP_EPOCH_MIN      ((time_t)1700000000)
#define GFSK_CONST_K        5.336446f   /* = pi*sqrt(2/ln2) */

static const char *TAG = "ft8_app";

static ft8_app_config_t s_cfg;
static bool s_utc_ok = false;

/* TX 运行状态(供 RX 每秒日志读取) */
static volatile bool s_tx_busy = false;
static volatile int64_t s_tx_next_us = 0;

/* 运行统计 */
static uint32_t s_stat_slots = 0;
static uint32_t s_stat_decoded = 0;

/* ---------- 协议参数换算 ---------- */
static bool app_is_ft4(void)             { return s_cfg.protocol == FTX_PROTOCOL_FT4; }
static int  app_sym_samples(void)        { return app_is_ft4() ? 576 : 1920; }
static int  app_nsym(void)               { return app_is_ft4() ? FT4_NN : FT8_NN; }
static int64_t app_slot_us(void)         { return app_is_ft4() ? (int64_t)7500000 : (int64_t)15000000; }

/* ============================================================
 * 发送消息：文本 -> payload -> tone 序列
 * ============================================================ */
static uint8_t s_tones[FT4_NN];

static esp_err_t tx_encode_message(void)
{
    ftx_message_t msg;
    char text[96];
    ftx_message_rc_t rc;

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
        ESP_LOGW(TAG, "FT8 无法表达 CQ %s，回退纯 CQ", s_cfg.cq_modifier);
    }
    snprintf(text, sizeof(text), "CQ %s %s", s_cfg.callsign, s_cfg.grid);
    rc = ftx_message_encode(&msg, NULL, text);
    if (rc != FTX_MESSAGE_RC_OK) {
        ESP_LOGE(TAG, "消息编码失败: %s", text);
        return ESP_FAIL;
    }
ok:
    if (app_is_ft4()) ft4_encode(msg.payload, s_tones);
    else              ft8_encode(msg.payload, s_tones);
    ESP_LOGI(TAG, "发射消息: %s", text);
    return ESP_OK;
}

/* ============================================================
 * GFSK 波形预生成(官方 gen_ft8 算法)
 * ============================================================ */
static int16_t *tx_build_wave(int nsym, int sym_samples, size_t *out_samples)
{
    const size_t S = (size_t)nsym * sym_samples;
    const int n_spsym = sym_samples;
    const int n_wave = (int)S;
    const float symbol_bt = app_is_ft4() ? 1.0f : 2.0f;

    int16_t *wave = heap_caps_malloc(S * sizeof(int16_t),
                                     MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (wave == NULL) wave = heap_caps_malloc(S * sizeof(int16_t), MALLOC_CAP_8BIT);
    if (wave == NULL) {
        ESP_LOGE(TAG, "发射波形内存不足(%u KB)", (unsigned)(S * 2 / 1024));
        return NULL;
    }

    const int len_dphi = n_wave + 2 * n_spsym;
    float *dphi = heap_caps_malloc((size_t)len_dphi * sizeof(float),
                                   MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (dphi == NULL) dphi = malloc((size_t)len_dphi * sizeof(float));
    float *pulse = malloc((size_t)3 * n_spsym * sizeof(float));
    if (dphi == NULL || pulse == NULL) {
        ESP_LOGE(TAG, "GFSK 工作内存不足");
        free(dphi); free(pulse); free(wave);
        return NULL;
    }

    const float dphi_carrier = 2.0f * (float)M_PI * s_cfg.audio_freq_hz / FT8_AUDIO_RATE;
    for (int i = 0; i < len_dphi; i++) dphi[i] = dphi_carrier;

    for (int i = 0; i < 3 * n_spsym; i++) {
        float t = i / (float)n_spsym - 1.5f;
        pulse[i] = (erff(GFSK_CONST_K * symbol_bt * (t + 0.5f)) -
                    erff(GFSK_CONST_K * symbol_bt * (t - 0.5f))) * 0.5f;
    }

    const float dphi_peak = 2.0f * (float)M_PI / n_spsym;
    for (int i = 0; i < nsym; i++) {
        int ib = i * n_spsym;
        for (int j = 0; j < 3 * n_spsym; j++) {
            dphi[j + ib] += dphi_peak * s_tones[i] * pulse[j];
        }
    }
    for (int j = 0; j < 2 * n_spsym; j++) {
        dphi[j]                  += dphi_peak * pulse[j + n_spsym] * s_tones[0];
        dphi[j + nsym * n_spsym] += dphi_peak * pulse[j]           * s_tones[nsym - 1];
    }

    const float scale = 32767.0f * s_cfg.audio_level;
    float phi = 0.0f;
    for (int k = 0; k < n_wave; k++) {
        wave[k] = (int16_t)(sinf(phi) * scale);
        phi = fmodf(phi + dphi[k + n_spsym], 2.0f * (float)M_PI);
    }

    int n_ramp = n_spsym / 8;
    for (int i = 0; i < n_ramp; i++) {
        float env = (1.0f - cosf(2.0f * (float)M_PI * i / (2 * n_ramp))) * 0.5f;
        wave[i]              = (int16_t)(wave[i] * env);
        wave[n_wave - 1 - i] = (int16_t)(wave[n_wave - 1 - i] * env);
    }

    free(pulse);
    free(dphi);
    *out_samples = S;
    ESP_LOGI(TAG, "发射波形生成完成(GFSK BT=%0.1f): %u 符号 %.2fs",
             symbol_bt, nsym, (double)S / FT8_AUDIO_RATE);
    return wave;
}

/* ---------------- 解码文本还原回调 ---------------- */
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
 * RX：每拍读一拍音频喂瀑布；时隙边界即整窗解析(去重)
 * ============================================================ */
static void rx_decode_slot(monitor_t *mon, int64_t prev_slot)
{
    const char *T = "ft8_app";
    const bool ft4 = app_is_ft4();

    if (!s_cfg.rx_enable) return;

    s_stat_slots++;
    const char *who = ((prev_slot & 1) == (s_cfg.tx_slot_parity & 1)) ? "本台发射时隙" : "对端接收时隙";
    ESP_LOGI(T, "[RX] 解析时隙 #%lld(%s) 结束，瀑布 %d/%d 块",
             (long long)prev_slot, who, mon->wf.num_blocks, mon->wf.max_blocks);

    /* 去重缓存 */
    static uint8_t seen[16][FTX_PAYLOAD_LENGTH_BYTES];
    static int seen_count = 0;
    static int seen_idx = 0;

    const float sym_period = ft4 ? FT4_SYMBOL_PERIOD : FT8_SYMBOL_PERIOD;
    const float bin_hz = 1.0f / sym_period;
    const int f_osr = (mon->wf.freq_osr > 0) ? mon->wf.freq_osr : 1;
    const int t_osr = (mon->wf.time_osr > 0) ? mon->wf.time_osr : 1;

    static ftx_candidate_t cands[140];
    int n = ftx_find_candidates(&mon->wf,
                                s_cfg.max_candidates > 0 ? s_cfg.max_candidates : 140,
                                cands, 10);
    for (int i = 0; i < n; i++) {
        ftx_message_t msg;
        ftx_decode_status_t st;
        if (!ftx_decode_candidate(&mon->wf, &cands[i],
                                  s_cfg.ldpc_iterations > 0 ? s_cfg.ldpc_iterations : 25,
                                  &msg, &st)) {
            continue;
        }
        char text[64];
        ftx_message_offsets_t offs;
        if (ftx_message_decode(&msg, &s_hash_if, text, &offs) != FTX_MESSAGE_RC_OK) {
            continue;
        }
        bool dup = false;
        for (int k = 0; k < seen_count; k++) {
            if (memcmp(seen[k], msg.payload, FTX_PAYLOAD_LENGTH_BYTES) == 0) {
                dup = true;
                break;
            }
        }
        if (dup) continue;
        memcpy(seen[seen_idx], msg.payload, FTX_PAYLOAD_LENGTH_BYTES);
        seen_idx = (seen_idx + 1) % 16;
        if (seen_count < 16) seen_count++;

        float f_hz = s_cfg.rx_f_min +
                     (cands[i].freq_offset + cands[i].freq_sub / (float)f_osr) * bin_hz;
        float t_s = (cands[i].time_offset + cands[i].time_sub / (float)t_osr) * sym_period;

        s_stat_decoded++;
        ESP_LOGI(T, "[RX] %s @%0.0fHz t=%0.2fs: %s",
                 ft4 ? "FT4" : "FT8", (double)f_hz, (double)t_s, text);
    }
    monitor_reset(mon);
}

static void ft8_rx_task(void *arg)
{
    (void)arg;
    const char *T = "ft8_app";
    const bool ft4 = app_is_ft4();
    const int sym_samples = app_sym_samples();
    const int64_t slot_us = app_slot_us();

    static int16_t ablk[MAX_SYMBOL_SAMPLES * 2];
    static float   fr[MAX_SYMBOL_SAMPLES];

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

    int64_t last_slot = -1;
    int64_t last_log_us = -1;

    for (;;) {
        /* 读一拍(全双工 RX，不因时隙/发射暂停) */
        int bytes = sym_samples * 4;
        int got = 0;
        while (got < bytes) {
            size_t rd = 0;
            if (wm8978_i2s_read((uint8_t *)ablk + got, bytes - got, &rd, 1000) != ESP_OK || rd == 0) {
                vTaskDelay(pdMS_TO_TICKS(5));
                continue;
            }
            got += (int)rd;
        }
        if (s_cfg.rx_enable) {
            for (int i = 0; i < sym_samples; i++) fr[i] = (float)ablk[i * 2] * (1.0f / 32768.0f);
            monitor_process(&mon, fr);
        }

        /* 时隙边界：上一时隙收齐 -> 解析 */
        int64_t now_us = esp_timer_get_time();
        int64_t slot = now_us / slot_us;
        if (slot != last_slot) {
            int64_t prev_slot = last_slot;
            last_slot = slot;
            if (s_cfg.rx_enable) rx_decode_slot(&mon, prev_slot);
        }

        /* 每秒状态 */
        if (now_us - last_log_us >= 1000000) {
            last_log_us = now_us;
            const char *clock = (s_cfg.utc_enable && s_utc_ok) ? "UTC" : "本地";
            const char *who = ((slot & 1) == (s_cfg.tx_slot_parity & 1)) ? "本台时隙" : "对端时隙";
            if (s_tx_busy) {
                ESP_LOGI(T, "[now] t=%lld.%03ds 时隙#%lld(%s) TX=发射中 RX=%d/%d块 已解%u(%u时隙) 时钟%s",
                         (long long)(now_us / 1000000), (int)((now_us / 1000) % 1000),
                         (long long)slot, who,
                         mon.wf.num_blocks, mon.wf.max_blocks,
                         (unsigned)s_stat_decoded, (unsigned)s_stat_slots, clock);
            } else {
                int64_t remain = (s_tx_next_us > now_us) ? (s_tx_next_us - now_us) / 1000 : 0;
                ESP_LOGI(T, "[now] t=%lld.%03ds 时隙#%lld(%s) TX=待机(下次%lldms) RX=%d/%d块 已解%u(%u时隙) 时钟%s",
                         (long long)(now_us / 1000000), (int)((now_us / 1000) % 1000),
                         (long long)slot, who, (long long)remain,
                         mon.wf.num_blocks, mon.wf.max_blocks,
                         (unsigned)s_stat_decoded, (unsigned)s_stat_slots, clock);
            }
        }
    }
}

/* ============================================================
 * TX：严格按目标时刻(esp_timer 本地栅格)发射整窗波形
 * ============================================================ */
static void ft8_tx_task(void *arg)
{
    (void)arg;
    const char *T = "ft8_app";
    const int sym_samples = app_sym_samples();
    const int nsym = app_nsym();
    const int64_t slot_us = app_slot_us();
    const int64_t msg_us = (int64_t)nsym * sym_samples * 1000000LL / FT8_AUDIO_RATE;
    const int64_t delay_us = (int64_t)s_cfg.tx_delay_ms * 1000;
    const int parity = s_cfg.tx_slot_parity & 1;
    const bool fits = (delay_us + msg_us <= slot_us);

    /* 预生成波形 */
    int16_t *wave = NULL;
    size_t wave_samples = 0;
    if (s_cfg.tx_enable && fits && tx_encode_message() == ESP_OK) {
        wave = tx_build_wave(nsym, sym_samples, &wave_samples);
    }
    if (s_cfg.tx_enable && wave == NULL) {
        ESP_LOGE(T, "发射不可用(编码失败或内存不足)，TX 关闭");
        s_cfg.tx_enable = false;
    }
    if (!s_cfg.tx_enable) {
        ESP_LOGI(T, "TX 已关闭，仅接收");
        vTaskDelete(NULL);
        return;
    }
    if (!fits) {
        ESP_LOGW(T, "发射延时+时长超时隙(%d.%03ds)，将跳过放不下的时隙",
                 (int)(delay_us / 1000000), (int)((delay_us / 1000) % 1000));
    }

    ESP_LOGI(T, "TX 任务启动: 奇偶时隙=%d 延时=%dms 时长=%.3fs",
             parity, (int)s_cfg.tx_delay_ms, (double)msg_us / 1000000.0);

    for (;;) {
        /* 找下一个“本台时隙”的可发射起点 */
        int64_t now = esp_timer_get_time();
        int64_t slot = now / slot_us;
        int64_t target = -1;
        for (int k = 0; k < 6; k++) {
            int64_t s = slot + k;
            if ((int)(s & 1) != parity) continue;              /* 只选本台时隙 */
            if (!fits) break;                                  /* 放不下则不发 */
            int64_t st = s * slot_us + delay_us;
            if (st + msg_us <= (s + 1) * slot_us && st > now + 30000) {
                target = st;
                break;
            }
        }
        if (target < 0) {          /* 本台时隙过近或放不下，等下一轮 */
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        s_tx_next_us = target;

        /* 先长睡到临近，再忙等到目标时刻，保证起播误差 <1ms */
        int64_t t = esp_timer_get_time();
        while (t < target - 30000) {
            vTaskDelay(pdMS_TO_TICKS(5));
            t = esp_timer_get_time();
        }
        while (esp_timer_get_time() < target) { /* busy wait */ }

    ESP_LOGI(T, "[TX] 时隙 #%lld 于 %d.%03ds 起播",
             (long long)(target / slot_us),
             (int)(target / 1000000), (int)((target / 1000) % 1000));
    s_tx_busy = true;

    /* 整窗播放：把单声道 wave 逐块复制成双声道交织(L=R)写入 */
    {
        static int16_t st[MAX_SYMBOL_SAMPLES * 2];
        size_t pos = 0;
        while (pos < wave_samples) {
            int n = (int)((wave_samples - pos) > MAX_SYMBOL_SAMPLES
                          ? MAX_SYMBOL_SAMPLES : (wave_samples - pos));
            for (int i = 0; i < n; i++) {
                st[i * 2 + 0] = wave[pos + i];
                st[i * 2 + 1] = wave[pos + i];
            }
            size_t bytes = (size_t)n * 4;
            size_t sent = 0;
            while (sent < bytes) {
                size_t wr = 0;
                esp_err_t err = wm8978_i2s_write((const uint8_t *)st + sent,
                                                 bytes - sent, &wr, 2000);
                if (err != ESP_OK || wr == 0) {
                    ESP_LOGW(T, "I2S 写异常");
                    vTaskDelay(pdMS_TO_TICKS(5));
                    continue;
                }
                sent += wr;
            }
            pos += (size_t)n;
        }
    }
    s_tx_busy = false;
    ESP_LOGI(T, "[TX] 发射完成，进入静默");
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
    cfg->utc_enable      = true;
    cfg->tx_slot_parity  = 0;
    cfg->tx_delay_ms     = 0;
    snprintf(cfg->callsign, sizeof(cfg->callsign), "BG7ABC");
    snprintf(cfg->grid,     sizeof(cfg->grid),     "JO70");
    cfg->msg_mode         = FT8_APP_MSG_CQ;
    cfg->cq_modifier[0]   = 0;
    cfg->audio_freq_hz    = 1200.0f;
    cfg->audio_level      = 0.45f;
    cfg->rx_f_min         = 0.0f;
    cfg->rx_f_max         = 4000.0f;
    cfg->rx_time_osr      = 2;
    cfg->rx_freq_osr      = 2;
    cfg->max_candidates   = 140;
    cfg->ldpc_iterations  = 25;
}

esp_err_t ft8_app_start(const ft8_app_config_t *cfg)
{
    static TaskHandle_t s_task_tx = NULL;
    static TaskHandle_t s_task_rx = NULL;
    if (cfg == NULL) return ESP_ERR_INVALID_ARG;
    if (s_task_rx != NULL) {
        ESP_LOGW(TAG, "FT8 应用已在运行，忽略本次启动");
        return ESP_OK;
    }
    s_cfg = *cfg;

    vTaskDelay(pdMS_TO_TICKS(200));
    if (WM8978_Init() != 0) {
        ESP_LOGE(TAG, "WM8978 初始化失败");
        return ESP_FAIL;
    }
    WM8978_ADDA_Cfg(1, 1);
    WM8978_Input_Cfg(1, 1, 0);
    WM8978_MIC_Gain(40);
    WM8978_Output_Cfg(1, 0);
    WM8978_I2S_Cfg(2, 0);
    WM8978_HPvol_Set(50, 50);
    WM8978_SPKvol_Set(40);
    ESP_ERROR_CHECK(wm8978_i2s_init(FT8_AUDIO_RATE));
    ESP_ERROR_CHECK(wm8978_i2s_start_rx());
    ESP_ERROR_CHECK(wm8978_i2s_start_tx());

    if (s_cfg.utc_enable) {
        s_utc_ok = (time(NULL) > SNTP_EPOCH_MIN);
        ESP_LOGW(TAG, "utc_enable: 系统时间%s校准；栅格严格对齐按本地 esp_timer，"
                 "UTC 亚秒相位需额外接入毫秒级墙钟",
                 s_utc_ok ? "已" : "未");
    }

    xTaskCreatePinnedToCore(ft8_rx_task, "ft8_rx", STACK_RX, NULL, 6, &s_task_rx, 1);
    xTaskCreatePinnedToCore(ft8_tx_task, "ft8_tx", STACK_TX, NULL, 6, &s_task_tx, 0);
    return ESP_OK;
}
