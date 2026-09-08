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
 *  RX 任务(ft8_rx)：每个时隙从时隙起点开始接收(喂瀑布)，到"时隙结束前
 *                  一段静默期"即停止接收并解析本时隙，解析不占用下一时隙
 *                  开头的采集窗口(与发射奇偶交替且紧贴时隙起点天然对齐)。
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

/* 配置为“引用”而非拷贝：ft8_app_start 只保存用户传入 cfg 的指针，任务持续
 * 读取它。运行中想换发射消息时，直接改 cfg.tx.type / call_to / rst_db 即可，
 * TX 任务下一个本台时隙前会自动重建波形，无需专用接口。 */
static ft8_app_config_t *s_cfg_p = NULL;
#define s_cfg (*s_cfg_p)

static bool s_utc_ok = false;

/* TX 运行状态(供 RX 每秒日志读取) */
static volatile bool s_tx_busy = false;
static volatile int64_t s_tx_next_us = 0;

/* TX 波形缓存：波形由“影响消息内容/波形的配置字段”决定，字段变了才重建 */
typedef struct {
    ft8_app_tx_msg_t tx;        /*!< 消息类型与参数(第几类消息+相关参数) */
    char callsign[16];          /*!< 本机呼号(消息文本用到) */
    char grid[8];               /*!< 本机网格 */
    float audio_freq_hz;        /*!< 波形载波 */
    float audio_level;          /*!< 波形幅度 */
} tx_wave_key_t;

static int16_t *s_wave = NULL;           /* 当前待播波形(PSRAM)，每个本台时隙播放 */
static size_t  s_wave_samples = 0;
static bool    s_wave_valid = false;     /* 有可用波形(可能沿用上一帧旧消息) */
static tx_wave_key_t s_wave_key;         /* 当前波形对应的配置字段快照 */
static tx_wave_key_t s_last_err_key;     /* 最近一次编码失败的快照(用于去重告警) */

/* 运行统计 */
static uint32_t s_stat_slots = 0;
static uint32_t s_stat_decoded = 0;

/* ---------- 协议参数换算 ---------- */
static bool app_is_ft4(void)             { return s_cfg.protocol == FTX_PROTOCOL_FT4; }
static int  app_sym_samples(void)        { return app_is_ft4() ? 576 : 1920; }
static int  app_nsym(void)               { return app_is_ft4() ? FT4_NN : FT8_NN; }
static int64_t app_slot_us(void)         { return app_is_ft4() ? (int64_t)7500000 : (int64_t)15000000; }

/* ============================================================
 * 发送消息：按 ft8_app_config_t.tx 决定第几类标准消息并取相关参数
 *           文本 -> payload -> tone 序列
 * ============================================================ */
static uint8_t s_tones[FT4_NN];

/* 读取“当前应发射内容”对应的配置字段快照(用户直接改 cfg 即反映到这里) */
static void tx_key_get(tx_wave_key_t *k)
{
    k->tx = s_cfg.tx;
    memcpy(k->callsign, s_cfg.callsign, sizeof(k->callsign));
    memcpy(k->grid,     s_cfg.grid,     sizeof(k->grid));
    k->audio_freq_hz = s_cfg.audio_freq_hz;
    k->audio_level   = s_cfg.audio_level;
}

/* 两个快照是否一致(决定波形是否需要重建)；整块比较，避免未清零字符串越界 */
static bool tx_key_eq(const tx_wave_key_t *a, const tx_wave_key_t *b)
{
    return memcmp(a, b, sizeof(*a)) == 0;
}

/* FT8 信号报告文本：±两位(如 -12 / +05)，R 报告加 "R" 前缀。
 * 该库报告字段(irpt=35+db)在 -31~-34 会撞上 RRR/RR73/73/空 等特殊值，
 * 因此只按 ±30 封顶(超出调用处已告警) */
static void tx_format_rst(char *buf, size_t n, int db, bool with_r)
{
    int mag = (db < 0) ? -db : db;
    if (mag > 30) mag = 30;
    char sign = (db < 0) ? '-' : '+';
    if (with_r) snprintf(buf, n, "R%c%02d", sign, mag);
    else        snprintf(buf, n, "%c%02d", sign, mag);
}

static esp_err_t tx_encode_message(const ft8_app_tx_msg_t *tx)
{
    ftx_message_t msg;
    char text[96];
    ftx_message_rc_t rc;

    if (tx->type != FT8_APP_MSG_CQ && !tx->call_to[0]) {
        ESP_LOGE(TAG, "消息类型 %d 需要设置目标呼号 tx.call_to", (int)tx->type);
        return ESP_FAIL;
    }

    ftx_message_init(&msg);
    switch (tx->type) {
    case FT8_APP_MSG_CQ:                                   /* 1. CQ 呼叫 */
        if (tx->cq_modifier[0]) {
            snprintf(text, sizeof(text), "CQ %s %s %s",
                     tx->cq_modifier, s_cfg.callsign, s_cfg.grid);
            rc = ftx_message_encode(&msg, NULL, text);
            if (rc == FTX_MESSAGE_RC_OK) goto ok;
            ESP_LOGW(TAG, "FT8 无法表达 CQ %s，回退纯 CQ", tx->cq_modifier);
        }
        snprintf(text, sizeof(text), "CQ %s %s", s_cfg.callsign, s_cfg.grid);
        break;

    case FT8_APP_MSG_CALL:                                 /* 2. 应答报网格 */
        snprintf(text, sizeof(text), "%s %s %s",
                 tx->call_to, s_cfg.callsign, s_cfg.grid);
        break;

    case FT8_APP_MSG_REPORT:                               /* 3. 信号报告 -10 */
    case FT8_APP_MSG_R_REPORT: {                           /* 4. R 回报告 R-12 */
        char rst[8];
        if (tx->rst_db > 30 || tx->rst_db < -30)
            ESP_LOGW(TAG, "rst_db=%d 超出库可编码范围(约±30)，按 ±30 发送", tx->rst_db);
        tx_format_rst(rst, sizeof(rst), tx->rst_db, tx->type == FT8_APP_MSG_R_REPORT);
        snprintf(text, sizeof(text), "%s %s %s", tx->call_to, s_cfg.callsign, rst);
        break;
    }

    case FT8_APP_MSG_RRR:                                  /* 5. RRR */
        snprintf(text, sizeof(text), "%s %s RRR", tx->call_to, s_cfg.callsign);
        break;
    case FT8_APP_MSG_RR73:                                 /* 5. RR73 */
        snprintf(text, sizeof(text), "%s %s RR73", tx->call_to, s_cfg.callsign);
        break;
    case FT8_APP_MSG_73:                                   /* 6. 73 */
        snprintf(text, sizeof(text), "%s %s 73", tx->call_to, s_cfg.callsign);
        break;

    default:
        ESP_LOGE(TAG, "未知 TX 消息类型 %d", (int)tx->type);
        return ESP_FAIL;
    }

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

/* ============================================================
 * TX 波形刷新：直接改的 cfg 影响发射内容时(消息类型/参数/呼号/网格/电平…)
 * 则重新编码并重建波形。只在时隙之间的空闲期执行(时间充裕)。编码失败时
 * 保留上一帧有效波形继续发射，避免出现"静默空拍"；返回 true = 有可用波形。
 * ============================================================ */
static bool tx_wave_refresh(void)
{
    tx_wave_key_t key;
    tx_key_get(&key);

    /* 与当前波形对应的配置一致 -> 无需重建 */
    if (s_wave_valid && tx_key_eq(&key, &s_wave_key)) return true;

    if (tx_encode_message(&key.tx) != ESP_OK) {
        if (!s_wave_valid || !tx_key_eq(&key, &s_last_err_key)) {
            ESP_LOGE(TAG, "TX 消息编码失败，%s",
                     s_wave_valid ? "沿用上一帧有效波形" : "无可用波形，TX 停发");
            s_last_err_key = key;
        }
        if (s_wave_valid) {
            s_wave_key = key;   /* 已尝试过本次配置，避免每轮循环重复尝试刷屏 */
            return true;
        }
        return false;
    }

    size_t n = 0;
    int16_t *w = tx_build_wave(app_nsym(), app_sym_samples(), &n);
    if (w == NULL) {
        ESP_LOGE(TAG, "发射波形内存不足");
        if (s_wave_valid) {
            s_wave_key = key;   /* 同上：已尝试过，沿用旧波形 */
            return true;
        }
        return false;
    }
    free(s_wave);
    s_wave = w;
    s_wave_samples = n;
    s_wave_key = key;
    s_wave_valid = true;
    return true;
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

/* 估算一条已解码消息的信号强度(近似 FT8/FT4 的 SNR，折算到 2500Hz 参考带宽)：
 *  - 信号 dB：整条消息在实际发送 tone 频点上的平均幅度；
 *  - 噪声 dB：同一符号内其余 n_tones-1 个 tone 频点的平均幅度(本地参考)；
 *  - 结果再减 10*log10(2500/bin_bw_hz)(FT8≈26dB，FT4≈21dB)折算到 2500Hz。
 * 仅用于日志/监视显示，非精确计量。解码失败或数据不足返回 NAN。 */
static float rx_measure_snr(const ftx_waterfall_t *wf, const ftx_candidate_t *cand,
                            const uint8_t *tones, int n_syms, int n_tones,
                            float bin_bw_hz)
{
    if (wf == NULL || wf->mag == NULL || cand == NULL || tones == NULL)
        return NAN;
    if (n_tones < 2 || bin_bw_hz <= 0.0f || cand->time_offset < 0 || cand->freq_offset < 0)
        return NAN;
    if (cand->freq_offset + n_tones > wf->num_bins)
        return NAN;

    int base = cand->time_offset;
    base = (base * wf->time_osr + cand->time_sub) * wf->freq_osr + cand->freq_sub;
    base = base * wf->num_bins + cand->freq_offset;

    double sum_sig = 0.0, sum_nse = 0.0;
    int n_sig = 0, n_nse = 0;

    for (int s = 0; s < n_syms; s++) {
        int block_abs = cand->time_offset + s;   /* 消息符号 s 对应的捕获块 */
        if (block_abs < 0 || block_abs >= wf->num_blocks) continue;
        const WF_ELEM_T *p = wf->mag + base + (size_t)s * wf->block_stride;
        int t = tones[s];                        /* 该符号实际发送的 tone 序号 */
        if (t < 0 || t >= n_tones) continue;
        sum_sig += WF_ELEM_MAG(p[t]);
        n_sig++;
        for (int b = 0; b < n_tones; b++) {
            if (b == t) continue;
            sum_nse += WF_ELEM_MAG(p[b]);
            n_nse++;
        }
    }
    if (n_sig < 4 || n_nse < n_sig)
        return NAN;

    /* 单 bin 噪声折算到 2500Hz 参考带宽：+10*log10(2500/bin_bw_hz) */
    float corr = 10.0f * log10f(2500.0f / bin_bw_hz);
    float snr_db = (float)(sum_sig / n_sig - sum_nse / n_nse) - corr;
    return snr_db;
}

/* ============================================================
 * RX：整窗解析一个时隙(带解码耗时预算，避免拖入下一时隙采集)
 * ============================================================ */
static void rx_decode_slot(monitor_t *mon, int64_t prev_slot, int64_t budget_us)
{
    const char *T = "ft8_app";
    const bool ft4 = app_is_ft4();

    if (!s_cfg.rx_enable) return;

    const int64_t t0 = esp_timer_get_time();

    s_stat_slots++;
    const char *who = ((prev_slot & 1) == (s_cfg.tx_slot_parity & 1)) ? "本台发射时隙" : "对端接收时隙";
    ESP_LOGI(T, "[RX] 解析时隙 #%lld(%s) 结束，瀑布 %d/%d 块",
             (long long)prev_slot, who, mon->wf.num_blocks, mon->wf.max_blocks);

    /* 去重缓存：容量跟随 cfg.max_candidates(上限 140，与候选数组一致)。
     * 放在函数内每次调用都是全新的 —— 即“每个时隙解析完自动作废/清空”，
     * 只在本时隙内对多个候选重复解出的同一条消息去重，不再跨时隙吞重复。 */
    int max_seen = (s_cfg.max_candidates > 0) ? s_cfg.max_candidates : 140;
    if (max_seen > 140) max_seen = 140;
    uint8_t seen[max_seen][FTX_PAYLOAD_LENGTH_BYTES];
    int seen_count = 0;
    int seen_idx = 0;

    const float sym_period = ft4 ? FT4_SYMBOL_PERIOD : FT8_SYMBOL_PERIOD;
    const float bin_hz = 1.0f / sym_period;
    const int f_osr = (mon->wf.freq_osr > 0) ? mon->wf.freq_osr : 1;
    const int t_osr = (mon->wf.time_osr > 0) ? mon->wf.time_osr : 1;

    static ftx_candidate_t cands[140];
    int n = ftx_find_candidates(&mon->wf,
                                s_cfg.max_candidates > 0 ? s_cfg.max_candidates : 140,
                                cands, 10);
    bool budget_cut = false;
    int  remaining = 0;
    for (int i = 0; i < n; i++) {
        if (esp_timer_get_time() - t0 > budget_us) {
            budget_cut = true;          /* 解析预算用尽：先保证下一时隙能按时开始接收 */
            remaining = n - i;
            break;
        }
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
        seen_idx = (seen_idx + 1) % max_seen;
        if (seen_count < max_seen) seen_count++;

        float f_hz = s_cfg.rx_f_min +
                     (cands[i].freq_offset + cands[i].freq_sub / (float)f_osr) * bin_hz;
        float t_s = (cands[i].time_offset + cands[i].time_sub / (float)t_osr) * sym_period;

        /* 用重编码得到的 tone 序列反查瀑布，估算信号强度(SNR, 2500Hz 带宽) */
        uint8_t tones[FT4_NN];
        if (ft4) ft4_encode(msg.payload, tones);
        else     ft8_encode(msg.payload, tones);
        float snr_db = rx_measure_snr(&mon->wf, &cands[i], tones,
                                      ft4 ? FT4_NN : FT8_NN, ft4 ? 4 : 8, bin_hz);
        char snr_str[16];
        if (isfinite(snr_db)) snprintf(snr_str, sizeof(snr_str), "%+.0fdB", snr_db);
        else                  snprintf(snr_str, sizeof(snr_str), "--dB");

        s_stat_decoded++;
        ESP_LOGI(T, "[RX] %s @%0.0fHz t=%0.2fs SNR=%s: %s",
                 ft4 ? "FT4" : "FT8", (double)f_hz, (double)t_s, snr_str, text);
    }
    if (budget_cut)
        ESP_LOGW(T, "[RX] 解析预算 %lldms 用尽提前结束，剩余 %d 个候选未处理",
                 (long long)(budget_us / 1000), remaining);
    monitor_reset(mon);
}

/* 丢弃一小段 RX 音频(在接收间隙也持续读取，防止 I2S DMA 积压，
 * 从而保证下个时隙一开始读到的是实时信号、窗口起点紧贴时隙边界) */
static void rx_discard_chunk(void)
{
    static uint8_t tmp[256];
    size_t rd = 0;
    (void)wm8978_i2s_read(tmp, sizeof(tmp), &rd, 2);
}

/* RX 每秒状态日志(供观察 TX/RX 节奏) */
static void rx_status_log(const monitor_t *mon, int64_t now_us, int64_t slot)
{
    static int64_t last_log_us = 0;
    const char *T = "ft8_app";
    if (last_log_us == 0) last_log_us = now_us;
    if (now_us - last_log_us < 1000000) return;
    last_log_us = now_us;

    const char *clock = (s_cfg.utc_enable && s_utc_ok) ? "UTC" : "本地";
    const char *who = ((slot & 1) == (s_cfg.tx_slot_parity & 1)) ? "本台时隙" : "对端时隙";
    if (s_tx_busy) {
        ESP_LOGI(T, "[now] t=%lld.%03ds 时隙#%lld(%s) TX=发射中 RX=%d/%d块 已解%u(%u时隙) 时钟%s",
                 (long long)(now_us / 1000000), (int)((now_us / 1000) % 1000),
                 (long long)slot, who,
                 mon->wf.num_blocks, mon->wf.max_blocks,
                 (unsigned)s_stat_decoded, (unsigned)s_stat_slots, clock);
    } else {
        int64_t remain = (s_tx_next_us > now_us) ? (s_tx_next_us - now_us) / 1000 : 0;
        ESP_LOGI(T, "[now] t=%lld.%03ds 时隙#%lld(%s) TX=待机(下次%lldms) RX=%d/%d块 已解%u(%u时隙) 时钟%s",
                 (long long)(now_us / 1000000), (int)((now_us / 1000) % 1000),
                 (long long)slot, who, (long long)remain,
                 mon->wf.num_blocks, mon->wf.max_blocks,
                 (unsigned)s_stat_decoded, (unsigned)s_stat_slots, clock);
    }
}

static void ft8_rx_task(void *arg)
{
    (void)arg;
    const char *T = "ft8_app";
    const bool ft4 = app_is_ft4();
    const int sym_samples = app_sym_samples();
    const int64_t slot_us = app_slot_us();
    const int64_t block_us = (int64_t)sym_samples * 1000000LL / FT8_AUDIO_RATE;

    static int16_t ablk[MAX_SYMBOL_SAMPLES * 2];
    static float   fr[MAX_SYMBOL_SAMPLES];

    monitor_t mon = { 0 };
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

    /* 时隙节奏：
     *  - 每个时隙从“时隙起点”开始接收(喂瀑布)；
     *  - 到“时隙结束前 parse_us”即停止接收，在末尾静默段解析本时隙，
     *    解析不占用下一时隙开头的采集窗口，窗口起点永远紧贴时隙边界。
     *  - parse_us 有下限(1.5s)，防止配置过小导致解码拖入下一时隙、
     *    进而整槽跳过(表现为“本台发射时隙 RX=0”)。 */
    const int64_t msg_us = (int64_t)app_nsym() * sym_samples * 1000000LL / FT8_AUDIO_RATE;
    const int64_t margin_us = 300000;                  /* 对端起播/本机发射延时容差 */
    int64_t parse_us = (int64_t)(s_cfg.rx_parse_ms > 0 ? s_cfg.rx_parse_ms : 1500) * 1000;
    if (parse_us < 1500000) parse_us = 1500000;        /* 解析期下限，保证每时隙都能按时解完 */
    const int64_t max_parse_us = slot_us - (msg_us + margin_us);
    if (parse_us > max_parse_us) parse_us = max_parse_us;   /* 不能挤占消息本身 */
    const int64_t cap_us = slot_us - parse_us;         /* 每时隙采集时长 */

    /* 解析预算：预留期再扣掉一个读块+余量，确保解码最晚在时隙边界前结束，
     * 下一时隙(含本台发射时隙)的接收不会被拖慢/跳过 */
    const int64_t guard_us = 300000;
    int64_t budget_us = parse_us - block_us - guard_us;
    if (budget_us < 100000) budget_us = 100000;
    ESP_LOGI(T, "%s RX 节奏: 前%.2fs接收, 末尾预留%.2fs解析(预算%.2fs)",
             ft4 ? "FT4" : "FT8",
             (double)cap_us / 1e6, (double)parse_us / 1e6, (double)budget_us / 1e6);

    for (;;) {
        /* 对齐到下一个时隙起点(期间读并丢弃旧音频，保持 DMA 不积压) */
        int64_t now = esp_timer_get_time();
        int64_t boundary = (now / slot_us + 1) * slot_us;
        while (esp_timer_get_time() < boundary) {
            rx_discard_chunk();
            rx_status_log(&mon, esp_timer_get_time(), boundary / slot_us - 1);
            vTaskDelay(pdMS_TO_TICKS(2));
        }

        const int64_t slot_id = boundary / slot_us;
        const int64_t cap_end = boundary + cap_us;

        /* 接收段：从时隙起点连续采集喂瀑布，直到 cap_end(时隙结束前 parse_us) */
        now = esp_timer_get_time();
        while (now < cap_end) {
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
            now = esp_timer_get_time();
            rx_status_log(&mon, now, slot_id);
        }

        /* 停止接收：在时隙末尾静默段解析本时隙(之后自动等到下一时隙起点) */
        rx_status_log(&mon, esp_timer_get_time(), slot_id);
        if (s_cfg.rx_enable) rx_decode_slot(&mon, slot_id, budget_us);

        int64_t after = esp_timer_get_time();
        int64_t next_b = (slot_id + 1) * slot_us;
        if (after > next_b) {
            ESP_LOGW(T, "解析耗时超时隙尾部%lldms，可能错过时隙 #%lld 开头",
                     (long long)((after - next_b) / 1000), (long long)(slot_id + 1));
        }
    }
}

/* ============================================================
 * TX：严格按目标时刻(esp_timer 本地栅格)发射整窗波形。
 *     每个本台时隙前读取 ft8_app_config_t.tx：运行中更改了"第几类消息"
 *     或相关参数，就在空闲期重新编码并重建波形，下一时隙自动生效。
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

    if (s_cfg.tx_enable && !fits) {
        ESP_LOGW(T, "发射延时+时长超时隙(%d.%03ds)，TX 关闭，仅接收",
                 (int)(delay_us / 1000000), (int)((delay_us / 1000) % 1000));
        s_cfg.tx_enable = false;
    }
    /* 启动时按初始配置预生成第一帧波形(失败则关闭 TX) */
    if (s_cfg.tx_enable && !tx_wave_refresh()) {
        ESP_LOGE(T, "发射不可用(编码失败或内存不足)，TX 关闭");
        s_cfg.tx_enable = false;
    }
    if (!s_cfg.tx_enable) {
        ESP_LOGI(T, "TX 已关闭，仅接收");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(T, "TX 任务启动: 奇偶时隙=%d 延时=%dms 时长=%.3fs",
             parity, (int)s_cfg.tx_delay_ms, (double)msg_us / 1000000.0);

    for (;;) {
        /* 运行时改了类型/参数 -> 空闲期重建波形(见 tx_wave_refresh) */
        if (!tx_wave_refresh()) {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        /* 找下一个“本台时隙”的可发射起点 */
        int64_t now = esp_timer_get_time();
        int64_t slot = now / slot_us;
        int64_t target = -1;
        for (int k = 0; k < 6; k++) {
            int64_t s = slot + k;
            if ((int)(s & 1) != parity) continue;              /* 只选本台时隙 */
            int64_t st = s * slot_us + delay_us;
            if (st + msg_us <= (s + 1) * slot_us && st > now + 30000) {
                target = st;
                break;
            }
        }
        if (target < 0) {          /* 本台时隙过近，等下一轮 */
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        s_tx_next_us = target;

        /* 先长睡到临近，再忙等到目标时刻，保证起播误差 <1ms。
         * 睡眠期间若发现 cfg 又被直接改过(热切换)，放弃本目标回外层重建，
         * 确保"运行中改配置，下一本台时隙生效"不失约 */
        int64_t t = esp_timer_get_time();
        bool stale = false;
        while (t < target - 30000) {
            vTaskDelay(pdMS_TO_TICKS(5));
            t = esp_timer_get_time();
            tx_wave_key_t cur;
            tx_key_get(&cur);
            if (!tx_key_eq(&cur, &s_wave_key)) {
                stale = true;
                break;
            }
        }
        if (stale) continue;
        while (esp_timer_get_time() < target) { /* busy wait */ }

        ESP_LOGI(T, "[TX] 时隙 #%lld 于 %d.%03ds 起播",
                 (long long)(target / slot_us),
                 (int)(target / 1000000), (int)((target / 1000) % 1000));
        s_tx_busy = true;

        /* 整窗播放：把单声道 s_wave 逐块复制成双声道交织(L=R)写入 */
        {
            static int16_t st[MAX_SYMBOL_SAMPLES * 2];
            size_t pos = 0;
            while (pos < s_wave_samples) {
                int n = (int)((s_wave_samples - pos) > MAX_SYMBOL_SAMPLES
                              ? MAX_SYMBOL_SAMPLES : (s_wave_samples - pos));
                for (int i = 0; i < n; i++) {
                    st[i * 2 + 0] = s_wave[pos + i];
                    st[i * 2 + 1] = s_wave[pos + i];
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

            /* 冲刷：DMA 环内还有 ~0.12s 残余，空闲时可能被循环重放，
             * 末尾补一整块全零，让残余与后续输出都固定为静音 */
            memset(st, 0, sizeof(st));
            {
                size_t bytes = sizeof(st);
                size_t sent = 0;
                while (sent < bytes) {
                    size_t wr = 0;
                    esp_err_t err = wm8978_i2s_write((const uint8_t *)st + sent,
                                                     bytes - sent, &wr, 2000);
                    if (err != ESP_OK || wr == 0) {
                        vTaskDelay(pdMS_TO_TICKS(5));
                        continue;
                    }
                    sent += wr;
                }
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
    cfg->tx.type           = FT8_APP_MSG_CQ;
    cfg->tx.rst_db         = -12;   /* 仅 REPORT / R_REPORT 用，默认给个常用值 */

    /* WM8978 初始化参数(对应原硬编码：ADDA(1,1) Input(1,1,0) MIC40 Out(1,0) I2S(2,0) HP(50,50) SPK40) */
    cfg->codec.dac_en     = 1;
    cfg->codec.adc_en     = 1;
    cfg->codec.mic_en     = 1;
    cfg->codec.linein_en  = 1;
    cfg->codec.aux_en     = 0;
    cfg->codec.mic_gain   = 40;
    cfg->codec.out_dac    = 1;
    cfg->codec.out_bypass = 0;
    cfg->codec.i2s_fmt    = 2;
    cfg->codec.i2s_len    = 0;
    cfg->codec.hp_vol_l   = 50;
    cfg->codec.hp_vol_r   = 50;
    cfg->codec.spk_vol    = 40;

    cfg->audio_freq_hz     = 1200.0f;
    cfg->audio_level      = 0.45f;
    cfg->rx_f_min         = 0.0f;
    cfg->rx_f_max         = 4000.0f;
    cfg->rx_time_osr      = 2;
    cfg->rx_freq_osr      = 2;
    cfg->max_candidates   = 60;  /*每时隙解码耗时 ≈ 候选数(max_candidates) × 每个候选迭代数(ldpc_iterations) × 单次迭代成本*/
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
    s_cfg_p = (ft8_app_config_t *)cfg;   /* 引用而非拷贝：运行中直接改 cfg 即生效 */

    vTaskDelay(pdMS_TO_TICKS(200));
    if (WM8978_Init() != 0) {
        ESP_LOGE(TAG, "WM8978 初始化失败");
        return ESP_FAIL;
    }
    WM8978_ADDA_Cfg(s_cfg.codec.dac_en, s_cfg.codec.adc_en);
    WM8978_Input_Cfg(s_cfg.codec.mic_en, s_cfg.codec.linein_en, s_cfg.codec.aux_en);
    WM8978_MIC_Gain(s_cfg.codec.mic_gain);
    WM8978_Output_Cfg(s_cfg.codec.out_dac, s_cfg.codec.out_bypass);
    WM8978_I2S_Cfg(s_cfg.codec.i2s_fmt, s_cfg.codec.i2s_len);
    WM8978_HPvol_Set(s_cfg.codec.hp_vol_l, s_cfg.codec.hp_vol_r);
    WM8978_SPKvol_Set(s_cfg.codec.spk_vol);
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
