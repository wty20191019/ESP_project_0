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
 * FT8/FT4 应用模块（单任务，按真实时序工作）
 *
 *  宏观层：时隙 = FT8 15s / FT4 7.5s，栅格可由 UTC(需 SNTP)对齐；
 *          台站只在 (时隙序号 % 2 == tx_slot_parity) 的时隙发射，
 *          其余时隙纯接收 —— 与对端自动奇偶交替。
 *  微观层：发射时隙内先放 79(或105) 个符号的有效波形，其后静默；
 *          发射波形一次生成存于 RAM(PSRAM)，逐符号流出。
 *
 *  调制：GFSK 高斯成形，算法对齐官方 ft8_lib/demo/gen_ft8.c：
 *        erf 差分高斯脉冲(BT: FT8=2.0 / FT4=1.0，长度 3 符号)，
 *        tone 序列卷积到每样本相位增量，头尾 dummy 符号补齐滤波边界，
 *        首尾 1/8 符号加余弦包络斜坡。tone0 = audio_freq_hz。
 *
 *  解码参数对齐官方 demo/decode_ft8.c：time_osr=2, freq_osr=2,
 *        min_score=10, 候选 140, LDPC 25 次。
 *
 *  主循环按"每符号一拍"：读一拍 ADC(兼作节拍) -> 喂瀑布 ->
 *        依时隙状态写有效波形或补零 -> 攒满一时隙后解码。
 * ============================================================ */

#define FT8_AUDIO_RATE      12000
#define MAX_SYMBOL_SAMPLES  1920
#define STACK_SIZE          32768

#define SNTP_EPOCH_MIN      ((time_t)1700000000)  /* 2023-11，判断时间是否已校时 */

#define GFSK_CONST_K        5.336446f   /* = pi*sqrt(2/ln2) */

static const char *TAG = "ft8_app";

static ft8_app_config_t s_cfg;
static TaskHandle_t s_task = NULL;

/* ---------- 协议参数换算 ---------- */
static bool app_is_ft4(void)             { return s_cfg.protocol == FTX_PROTOCOL_FT4; }
static int  app_sym_samples(void)        { return app_is_ft4() ? 576 : 1920; }
static int  app_nsym(void)               { return app_is_ft4() ? FT4_NN : FT8_NN; }
static int64_t app_slot_ms(void)         { return app_is_ft4() ? 7500 : 15000; }

/* 当前"墙钟毫秒"：UTC 校时后取 UTC，否则退化为上电时刻本地栅格 */
static bool s_utc_ok = false;
static int64_t wall_ms(void)
{
    if (s_cfg.utc_enable && s_utc_ok) {
        return (int64_t)time(NULL) * 1000;
    }
    return esp_timer_get_time() / 1000;         /* 本地毫秒(自启动) */
}

/* ============================================================
 * 发送消息：文本 -> payload -> tone 序列
 * ============================================================ */
static uint8_t s_tones[FT4_NN];          /* 灰码 tone 序列(取较大者) */

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
 * 发射波形预生成：GFSK 高斯成形（对齐官方 demo/gen_ft8.c）
 * ============================================================ */
static int16_t *tx_build_wave(int nsym, int sym_samples, size_t *out_samples)
{
    const size_t S = (size_t)nsym * sym_samples;      /* 单声道样本总数 */
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

    /* 相位增量工作区：波形 + 头尾各延长一个符号 */
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

    /* 1. 载波相位增量 */
    const float dphi_carrier = 2.0f * (float)M_PI * s_cfg.audio_freq_hz / FT8_AUDIO_RATE;
    for (int i = 0; i < len_dphi; i++) dphi[i] = dphi_carrier;

    /* 2. 高斯成形脉冲(erf 差分) */
    for (int i = 0; i < 3 * n_spsym; i++) {
        float t = i / (float)n_spsym - 1.5f;
        pulse[i] = (erff(GFSK_CONST_K * symbol_bt * (t + 0.5f)) -
                    erff(GFSK_CONST_K * symbol_bt * (t - 0.5f))) * 0.5f;
    }

    /* 3. tone 序列卷积叠加(频偏步进 = fs/n_spsym) */
    const float dphi_peak = 2.0f * (float)M_PI / n_spsym;
    for (int i = 0; i < nsym; i++) {
        int ib = i * n_spsym;
        for (int j = 0; j < 3 * n_spsym; j++) {
            dphi[j + ib] += dphi_peak * s_tones[i] * pulse[j];
        }
    }
    /* 头尾 dummy 符号：用首/末符号补齐滤波边界 */
    for (int j = 0; j < 2 * n_spsym; j++) {
        dphi[j]                  += dphi_peak * pulse[j + n_spsym] * s_tones[0];
        dphi[j + nsym * n_spsym] += dphi_peak * pulse[j]           * s_tones[nsym - 1];
    }

    /* 4. 相位积分生成波形(丢弃 dummy 段) */
    const float scale = 32767.0f * s_cfg.audio_level;
    float phi = 0.0f;
    for (int k = 0; k < n_wave; k++) {
        wave[k] = (int16_t)(sinf(phi) * scale);
        phi = fmodf(phi + dphi[k + n_spsym], 2.0f * (float)M_PI);
    }

    /* 5. 首尾 1/8 符号余弦包络斜坡 */
    int n_ramp = n_spsym / 8;
    for (int i = 0; i < n_ramp; i++) {
        float env = (1.0f - cosf(2.0f * (float)M_PI * i / (2 * n_ramp))) * 0.5f;
        wave[i]            = (int16_t)(wave[i] * env);
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

/* 运行统计(供每秒状态日志) */
static uint32_t s_stat_slots = 0;       /* 已完成解码的时隙数 */
static uint32_t s_stat_decoded = 0;     /* 成功解码消息条数 */

/* ============================================================
 * 接收解码：每次时隙(含本台发射时隙)结束/边界变化时调用，解析刚
 * 结束时段积累的瀑布。解码结果按 payload 去重，避免刷屏。
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

    /* 消息去重：同一 payload(同一消息)只报第一次 */
    static uint8_t seen[16][FTX_PAYLOAD_LENGTH_BYTES];
    static int seen_count = 0;
    static int seen_idx = 0;

    /* 本库 st.freq/time 未填，用候选索引换算估计频率/时间 */
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

        /* 去重 */
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
    monitor_reset(mon);     /* 清空，开始接收下一时隙 */
}

/* ============================================================
 * 主任务
 * ============================================================ */
static void ft8_app_task(void *arg)
{
    (void)arg;
    const char *T = "ft8_app";

    const bool ft4 = app_is_ft4();
    const int sym_samples = app_sym_samples();
    const int nsym = app_nsym();
    const int64_t slot_ms = app_slot_ms();
    const int64_t msg_ms = (int64_t)nsym * app_sym_samples() * 1000 / FT8_AUDIO_RATE;

    static int16_t ablk[MAX_SYMBOL_SAMPLES * 2];    /* 收发立体声块 */
    static float   fr[MAX_SYMBOL_SAMPLES];          /* 单声道浮点帧 */

    /* --- UTC 状态(宏观层) --- */
    if (s_cfg.utc_enable) {
        s_utc_ok = (time(NULL) > SNTP_EPOCH_MIN);
        if (!s_utc_ok) {
            ESP_LOGW(T, "系统时间未校准(需 SNTP)，退化为本地栅格");
        } else {
            ESP_LOGI(T, "系统时间已校准，时隙按 UTC 对齐");
        }
    }

    /* --- 发射波形(微观层)：预生成，供本台时隙重复使用 --- */
    int16_t *tx_wave = NULL;
    size_t tx_wave_samples = 0;
    const bool tx_fits = (int64_t)s_cfg.tx_delay_ms + msg_ms <= slot_ms;
    if (s_cfg.tx_enable) {
        if (!tx_fits) {
            ESP_LOGW(T, "发射延时+消息时长超时隙，将自动跳过放不下的时隙");
        }
        if (tx_encode_message() == ESP_OK) {
            tx_wave = tx_build_wave(nsym, sym_samples, &tx_wave_samples);
        }
    }
    if (s_cfg.tx_enable && tx_wave == NULL) {
        s_cfg.tx_enable = false;
    }

    /* --- 解码瀑布 --- */
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

    /* --- TX 时隙状态机 --- */
    int64_t last_slot = -1;
    bool tx_active = false;
    bool tx_pending = false;
    int64_t tx_start_ms = 0;
    int tx_idx = 0;
    bool tx_this_slot = false;
    int64_t slot_start_ms = 0;
    int64_t last_log_ms = -1;   /* 每秒状态日志节流 */

    ESP_LOGI(T, "%s 应用启动: 收=%d 发=%d(奇偶时隙=%d, 延时=%ums) 音频=%0.0fHz 电平=%0.2f",
             ft4 ? "FT4" : "FT8", s_cfg.rx_enable, s_cfg.tx_enable,
             s_cfg.tx_slot_parity & 1, (unsigned)s_cfg.tx_delay_ms,
             (double)s_cfg.audio_freq_hz, (double)s_cfg.audio_level);

    /* ==================== 主循环(每符号一拍) ==================== */
    for (;;) {
        /* 1. 读一拍音频 */
        int bytes = sym_samples * 4;
        int got = 0;
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

        /* 3. 宏观层：时隙推进。音频采集不因时隙/发射而中断；
         *    一旦跨过边界，先用刚结束时隙的瀑布解析一次，再切 TX 状态 */
        int64_t now_ms = wall_ms();
        int64_t slot = now_ms / slot_ms;
        if (slot != last_slot) {
            int64_t prev_slot = last_slot;   /* 刚结束的时隙(-1 为首窗) */
            last_slot = slot;

            if (s_cfg.rx_enable) {
                rx_decode_slot(&mon, prev_slot);   /* 无论什么时隙结束都解析 */
            }

            slot_start_ms = slot * slot_ms;
            tx_this_slot = (s_cfg.tx_enable) && (((int)(slot & 1)) == (s_cfg.tx_slot_parity & 1));
            tx_active = false;
            tx_pending = false;
            tx_idx = 0;
            if (tx_this_slot && tx_fits) {
                tx_start_ms = slot_start_ms + s_cfg.tx_delay_ms;
                tx_pending = true;
                ESP_LOGI(T, "[TX] 本台发射时隙 #%lld 于 %d.%03ds 开始",
                         (long long)slot,
                         (int)(tx_start_ms / 1000), (int)(tx_start_ms % 1000));
            } else if (tx_this_slot && !tx_fits) {
                ESP_LOGI(T, "[TX] 时隙 #%lld 发射放不下，跳过", (long long)slot);
            }
        }
        if (tx_pending && now_ms >= tx_start_ms) {
            tx_pending = false;
            tx_active = true;
            tx_idx = 0;
        }

        /* 4. 微观层：本拍输出有效波形段或补零 */
        bool emit = tx_active && (tx_idx < nsym) && (tx_wave != NULL);
        if (emit) {
            const int16_t *seg = &tx_wave[(size_t)tx_idx * sym_samples];
            for (int i = 0; i < sym_samples; i++) {
                ablk[i * 2 + 0] = seg[i];
                ablk[i * 2 + 1] = seg[i];
            }
            if (++tx_idx >= nsym) {
                tx_active = false;
                ESP_LOGI(T, "[TX] 本时隙发射完成，进入静默间隙");
            }
        } else {
            memset(ablk, 0, (size_t)sym_samples * 4);   /* 静默(解码/T-R 间隙) */
        }

        /* 5. 写出本拍音频 */
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

        /* 6. 每秒状态提醒：时间/时隙归属/收发/解码统计 */
        if (now_ms - last_log_ms >= 1000) {
            last_log_ms = now_ms;
            const char *clock = s_utc_ok ? "UTC" : "本地";
            const char *who = ((slot & 1) == (s_cfg.tx_slot_parity & 1)) ? "本台时隙" : "对端时隙";
            const char *tx_st = !s_cfg.tx_enable ? "关" :
                                (tx_pending ? "等待发射" :
                                (tx_active ? "发射中" : "静默"));
            if (s_cfg.tx_enable && tx_active) {
                ESP_LOGI(T, "[now] t=%lld.%03ds 时隙#%lld(%s) TX=发射中(%d/%d) RX=%d/%d块 已解%u(%u时隙) 时钟%s",
                         (long long)(now_ms / 1000), (int)(now_ms % 1000),
                         (long long)slot, who,
                         tx_idx, nsym,
                         mon.wf.num_blocks, mon.wf.max_blocks,
                         (unsigned)s_stat_decoded, (unsigned)s_stat_slots, clock);
            } else {
                ESP_LOGI(T, "[now] t=%lld.%03ds 时隙#%lld(%s) TX=%s RX=%d/%d块 已解%u(%u时隙) 时钟%s",
                         (long long)(now_ms / 1000), (int)(now_ms % 1000),
                         (long long)slot, who, tx_st,
                         mon.wf.num_blocks, mon.wf.max_blocks,
                         (unsigned)s_stat_decoded, (unsigned)s_stat_slots, clock);
            }
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
    cfg->utc_enable      = true;       /* 需先 SNTP 校时；未校时自动回退本地栅格 */
    cfg->tx_slot_parity  = 0;          /* 偶数时隙发射(WSJT even)，奇数接收 */
    cfg->tx_delay_ms     = 0;
    snprintf(cfg->callsign, sizeof(cfg->callsign), "BG7ABC");
    snprintf(cfg->grid,     sizeof(cfg->grid),     "JO70");
    cfg->msg_mode         = FT8_APP_MSG_CQ;
    cfg->cq_modifier[0]   = 0;
    cfg->audio_freq_hz    = 1200.0f;
    cfg->audio_level      = 0.45f;
    cfg->rx_f_min         = 0.0f;
    cfg->rx_f_max         = 4000.0f;
    cfg->rx_time_osr      = 2;         /* 对齐官方 demo */
    cfg->rx_freq_osr      = 2;
    cfg->max_candidates   = 140;
    cfg->ldpc_iterations  = 25;
}

esp_err_t ft8_app_start(const ft8_app_config_t *cfg)
{
    if (cfg == NULL) return ESP_ERR_INVALID_ARG;
    if (s_task != NULL) {
        ESP_LOGW(TAG, "FT8 应用已在运行，忽略本次启动");
        return ESP_OK;
    }
    s_cfg = *cfg;

    /* 音频通路(W-M8978 + I2S0 全双工 12kHz)，模块内幂等初始化 */
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

    xTaskCreatePinnedToCore(ft8_app_task, "ft8_app", STACK_SIZE, NULL, 6, &s_task, 1);
    return ESP_OK;
}
