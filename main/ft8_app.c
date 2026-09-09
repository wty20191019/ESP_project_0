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
#include "freertos/queue.h"
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
 *  ⚠ 时隙栅格：utc_enable+gps_utc_enable 且 GPS UTC+PPS 就绪时，用 GPS 把
 *    esp_timer 相位映射到 UTC，按 UTC(:00/:15/:30/:45) 栅格收发；GPS 未
 *    就绪时回退到以 esp_timer(上电时刻)为 0 的本地栅格(仅用于无网测试)。
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
 * UTC 栅格映射：esp_timer ↔ GPS UTC
 * 用 GPS 的 PPS 上升沿(esp_timer 时刻)配合同一时刻的 UTC 时间，得到
 * “esp_timer 时刻 ↔ UTC 秒”的固定相位关系，之后 RX/TX 都按 UTC 的
 * 15s/7.5s 栅格(整分钟 :00/:15/:30/:45)计算时隙，实现真正 UTC 对齐。
 * 校准只在 GPS UTC+PPS 首次有效时锁存一次(esp_timer 晶振漂移可忽略)。
 * ============================================================ */
static portMUX_TYPE s_utc_mux = portMUX_INITIALIZER_UNLOCKED;
static bool     s_utc_locked = false;      /* 已用 GPS 校准 UTC 相位 */
static int64_t  s_utc_edge_us = 0;         /* 校准点：PPS 上升沿的 esp_timer µs */
static int64_t  s_utc_edge_sec = 0;        /* 校准点：该沿对应的 UTC 秒(1970 起) */

/* 儒略日(公历) -> 自 1970-01-01 的天数 */
static int64_t utc_days_from_civil(int64_t y, int64_t m, int64_t d)
{
    y -= m <= 2;
    int64_t era = (y >= 0 ? y : y - 399) / 400;
    unsigned yoe = (unsigned)(y - era * 400);
    unsigned doy = (153u * (unsigned)(m + (m > 2 ? -3 : 9)) + 2) / 5u + (unsigned)d - 1u;
    unsigned doe = yoe * 365u + yoe / 4u - yoe / 100u + doy;
    return era * 146097 + (int64_t)doe - 719468;
}

static int64_t utc_to_epoch_s(const ft8_app_gps_time_t *g)
{
    int64_t days = utc_days_from_civil(g->year, g->month, g->day);
    return days * 86400 + g->hour * 3600 + g->minute * 60 + g->second;
}

typedef struct {
    bool    locked;
    int64_t edge_us;
    int64_t edge_sec;
} utc_ref_t;

static void utc_ref_get(utc_ref_t *r)
{
    portENTER_CRITICAL(&s_utc_mux);
    r->locked    = s_utc_locked;
    r->edge_us   = s_utc_edge_us;
    r->edge_sec  = s_utc_edge_sec;
    portEXIT_CRITICAL(&s_utc_mux);
}

/* esp_timer now_us -> UTC 微秒(自1970)；未锁定时原样返回(本地栅格) */
static int64_t utc_to_us(const utc_ref_t *r, int64_t now_us)
{
    if (!r->locked) return now_us;
    return r->edge_sec * 1000000LL + (now_us - r->edge_us);
}

/* UTC 微秒 -> esp_timer 微秒 */
static int64_t utc_from_us(const utc_ref_t *r, int64_t utc_us)
{
    if (!r->locked) return utc_us;
    return r->edge_us + (utc_us - r->edge_sec * 1000000LL);
}

static bool gps_cfg_eq(const ft8_app_gps_time_t *a, const ft8_app_gps_time_t *b)
{
    return a->valid == b->valid &&
           a->year == b->year && a->month == b->month && a->day == b->day &&
           a->hour == b->hour && a->minute == b->minute && a->second == b->second &&
           a->millisecond == b->millisecond && a->pps_seq == b->pps_seq &&
           a->pps_edge_us == b->pps_edge_us;
}

/* 稳定读取 cfg.gps(避免另一核写入时读到撕裂的 64 位字段) */
static bool gps_cfg_get(ft8_app_gps_time_t *out)
{
    ft8_app_gps_time_t a, b;
    for (int i = 0; i < 4; i++) {
        a = s_cfg.gps;
        b = s_cfg.gps;
        if (gps_cfg_eq(&a, &b)) {
            *out = a;
            return a.valid;
        }
    }
    return false;
}

/* 若启用 GPS UTC 且 GPS 时间已就绪，则锁存一次 UTC 相位。
 *  - gps_use_pps=true：用 PPS 上升沿做亚秒精对齐；为排除“PPS 已进到下一秒而
 *    NMEA 时间还没更新”的短暂错位，要求两次采样 pps_seq 与 UTC 秒同步前进。
 *  - gps_use_pps=false：不用 PPS，把解析到该 UTC 秒的 esp_timer 时刻近似当作
 *    该秒起点(NMEA 粗对齐，误差可达数百 ms~1s，仅适合无 PPS 场合)。 */
static void utc_try_lock_gps(void)
{
    if (s_utc_locked) return;
    if (!s_cfg.utc_enable || !s_cfg.gps_utc_enable) return;

    const bool use_pps = s_cfg.gps_use_pps;

    ft8_app_gps_time_t g;
    if (!gps_cfg_get(&g) || !g.valid) return;
    if (use_pps && g.pps_seq == 0) return;              /* 要求 PPS 但尚未收到 */

    static uint32_t prev_seq = 0;
    static uint8_t  prev_sec = 0;
    static bool     have_prev = false;

    if (use_pps) {
        if (!have_prev) {
            prev_seq = g.pps_seq;
            prev_sec = g.second;
            have_prev = true;
            return;
        }
        uint32_t dseq = g.pps_seq - prev_seq;           /* PPS 前进步数 */
        int dsec = (int)g.second - (int)prev_sec;
        if (dsec < 0) dsec += 60;                       /* 跨分钟回绕 */
        prev_seq = g.pps_seq;
        prev_sec = g.second;
        if (dseq < 1 || dsec != (int)dseq) return;      /* 时间与 PPS 未同步前进 */
    }

    int64_t sec = utc_to_epoch_s(&g);
    if (sec < 946684800LL) return;                      /* 早于 2000-01-01 视为无效 */

    /* PPS 精对齐：校准点取 PPS 上升沿；NMEA 粗对齐：校准点取“当前解析时刻” */
    int64_t edge_us = use_pps ? g.pps_edge_us : esp_timer_get_time();
    bool    locked_now = false;
    portENTER_CRITICAL(&s_utc_mux);
    if (!s_utc_locked) {
        s_utc_locked    = true;
        s_utc_edge_us   = edge_us;
        s_utc_edge_sec  = sec;
        s_utc_ok        = true;
        locked_now      = true;
    }
    portEXIT_CRITICAL(&s_utc_mux);

    if (locked_now) {
        ESP_LOGI(TAG, "GPS UTC 相位已校准(%s): %04u-%02u-%02u %02u:%02u:%02u.%03u "
                 "PPS#%lu edge=%lldus",
                 use_pps ? "PPS" : "NMEA粗对齐",
                 g.year, g.month, g.day, g.hour, g.minute, g.second, g.millisecond,
                 (unsigned long)g.pps_seq, (long long)edge_us);
    }
}

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

/* RX 解码成功后将结构化解码消息投递给自动 QSO 引擎(定义见文件尾部) */
static void qso_rx_publish(const ftx_message_t *msg, float freq_hz, float snr_db, int64_t slot);

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
    if (n_tones < 2 || bin_bw_hz <= 0.0f || cand->freq_offset < 0)
        return NAN;
    if (cand->freq_offset + n_tones > wf->num_bins)
        return NAN;

    /* time_offset 允许为负: 消息起点可能早于本时隙采集窗口(前一两个符号溢出到
     * 上一时隙)。这类符号没有数据, 下面按 block_abs 跳过; 只要窗内符号够多
     * (n_sig>=4) 仍可估算, 不再因起点略早而整体放弃。 */

    double sum_sig = 0.0, sum_nse = 0.0;
    int n_sig = 0, n_nse = 0;

    for (int s = 0; s < n_syms; s++) {
        int block_abs = cand->time_offset + s;   /* 消息符号 s 对应的捕获块 */
        if (block_abs < 0 || block_abs >= wf->num_blocks) continue;
        /* 布局: mag[块][time_osr][freq_osr][num_bins]; 取该块子采样/频偏位置 */
        int within = cand->time_sub * (wf->freq_osr * wf->num_bins) +
                     cand->freq_sub * wf->num_bins + cand->freq_offset;
        const WF_ELEM_T *p = wf->mag +
                             (size_t)block_abs * (size_t)wf->block_stride +
                             (size_t)within;
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

        /* 投递给自动 QSO 引擎(启用时才建队, 无队则此调用为空操作) */
        qso_rx_publish(&msg, f_hz, snr_db, prev_slot);

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

    const char *clock = s_utc_locked ? "UTC(GPS)"
                     : (s_utc_ok ? "UTC(SNTP)" : "本地");
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

/* ============================================================
 * 瀑布显示快照(RX 每收到一个符号块, 压缩成一行功率谱, 供 LCD 绘制)
 * 写入端: ft8_rx_task(core1); 读取端: LCD 任务(core0)。
 * 无锁设计: 读端容忍看到一帧正在写入的行(写入 160ms 才一行, 风险可忽略)。
 * ============================================================ */
static ft8_wf_snap_t *s_wf = NULL;

static void wf_snap_alloc(void)
{
    if (s_wf) return;
    ft8_wf_snap_t *w = heap_caps_malloc(sizeof(*w), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (w == NULL) w = malloc(sizeof(*w));
    if (w) {
        memset(w, 0, sizeof(*w));
        s_wf = w;
    } else {
        ESP_LOGE(TAG, "瀑布快照内存不足, 瀑布页将无数据");
    }
}

/* 把 monitor 刚写完的符号块(num_blocks-1)压缩成一行 FT8_WF_COLS 点功率谱。
 * mag 布局: [块][time_osr][freq_osr][num_bins], 值 0..255(≈-120..0dB)。 */
static void wf_disp_add(const ftx_waterfall_t *wf)
{
    if (wf == NULL || wf->mag == NULL) return;
    if (wf->num_blocks <= 0 || wf->num_blocks > wf->max_blocks) return;
    if (wf->num_bins <= 0) return;

    if (s_wf == NULL) wf_snap_alloc();
    if (s_wf == NULL) return;

    const int nb     = wf->num_bins;
    const int stride = wf->block_stride;
    const int t_osr  = wf->time_osr > 0 ? wf->time_osr : 1;
    const int f_osr  = wf->freq_osr > 0 ? wf->freq_osr : 1;
    const int os     = (wf->num_blocks - 1) * stride;   /* 最新块的起始 */

    uint8_t *row = s_wf->rows[s_wf->put];
    for (int c = 0; c < FT8_WF_COLS; c++) {
        int lo = (c * nb) / FT8_WF_COLS;
        int hi = ((c + 1) * nb) / FT8_WF_COLS;
        if (hi <= lo) hi = lo + 1;
        if (lo >= nb) { row[c] = 0; continue; }
        if (hi > nb) hi = nb;

        uint8_t mx = 0;
        for (int b = lo; b < hi; b++) {                 /* 列内 bin(取峰值) */
            for (int t = 0; t < t_osr; t++) {
                int base = os + (t * f_osr) * nb;
                for (int f = 0; f < f_osr; f++) {
                    uint8_t v = wf->mag[base + f * nb + b];
                    if (v > mx) mx = v;
                }
            }
        }
        row[c] = mx;
    }
    s_wf->put = (s_wf->put + 1) % FT8_WF_ROWS;
    s_wf->seq++;
}

const ft8_wf_snap_t *ft8_wf_snap(void)
{
    return s_wf;
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
        /* 尝试用 GPS UTC+PPS 锁存 UTC 相位(仅在启用且 GPS 就绪后锁一次) */
        utc_try_lock_gps();
        utc_ref_t ref;
        utc_ref_get(&ref);

        /* 对齐到“下一个时隙起点”：UTC 锁定时为 UTC 栅格(:00/:15/:30/:45)，
         * 否则退回本地 esp_timer 栅格(此时 ref 映射是恒等) */
        int64_t now = esp_timer_get_time();
        int64_t now_utc = utc_to_us(&ref, now);
        int64_t cur_slot = now_utc / slot_us;
        int64_t boundary = utc_from_us(&ref, (cur_slot + 1) * slot_us);
        while (esp_timer_get_time() < boundary) {
            rx_discard_chunk();
            rx_status_log(&mon, esp_timer_get_time(), cur_slot);
            vTaskDelay(pdMS_TO_TICKS(2));
        }

        const int64_t slot_id = cur_slot + 1;   /* 即将接收的时隙号(UTC/本地) */
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
                wf_disp_add(&mon.wf);      /* 新符号入瀑布, 供 LCD 页绘制 */
            }
            now = esp_timer_get_time();
            rx_status_log(&mon, now, slot_id);
        }

        /* 停止接收：在时隙末尾静默段解析本时隙(之后自动等到下一时隙起点) */
        rx_status_log(&mon, esp_timer_get_time(), slot_id);
        if (s_cfg.rx_enable) rx_decode_slot(&mon, slot_id, budget_us);

        int64_t after = esp_timer_get_time();
        int64_t next_b = utc_from_us(&ref, (slot_id + 1) * slot_us);
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

    ESP_LOGI(T, "TX 任务启动(自动 QSO 引擎可运行中开关/切相位)");

    for (;;) {
        /* ---- 每轮实时读取, 支持自动 QSO 引擎在运行中开关/改奇偶/改内容 ---- */
        const int parity = s_cfg.tx_slot_parity & 1;
        const int64_t delay_us = (int64_t)s_cfg.tx_delay_ms * 1000;
        const bool fits = (delay_us + msg_us <= slot_us);

        if (!s_cfg.tx_enable || !fits) {
            vTaskDelay(pdMS_TO_TICKS(250));
            continue;
        }

        /* 运行时改了类型/参数 -> 空闲期重建波形(见 tx_wave_refresh) */
        if (!tx_wave_refresh()) {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        /* 找下一个“本台时隙”的可发射起点(UTC 锁定时按 UTC 栅格/奇偶) */
        utc_try_lock_gps();
        utc_ref_t ref;
        utc_ref_get(&ref);
        int64_t now = esp_timer_get_time();
        int64_t now_utc = utc_to_us(&ref, now);
        int64_t slot = now_utc / slot_us;
        int64_t target = -1;
        for (int k = 0; k < 6; k++) {
            int64_t s = slot + k;
            if ((int)(s & 1) != parity) continue;              /* 只选本台时隙 */
            int64_t st = utc_from_us(&ref, s * slot_us) + delay_us;
            int64_t se = utc_from_us(&ref, (s + 1) * slot_us);
            if (st + msg_us <= se && st > now + 30000) {
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
                 (long long)(utc_to_us(&ref, target) / slot_us),
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
 * 自动 QSO 引擎(一个真正的 FT8/FT4 通联状态机)
 *
 * 数据流:
 *   RX 任务在 rx_decode_slot 把每条解码成功的消息结构化成 qso_rx_t
 *   (标准消息三字段 call_to/call_de/extra + 字段类型 + 频率/SNR/时隙),
 *   经 s_qso_q 队列投递到本引擎任务。
 *   引擎按"发射方视角"识别消息语义并驱动状态机, 通过改写 cfg.tx /
 *   cfg.tx_enable / cfg.tx_slot_parity 让 TX 任务在下一个本台时隙发射
 *   (复用现成的"热切换"机制, 引擎不直接操作音频)。
 *
 * FT8 通联约定(标准消息恒为 "<目标> <发射方> <第三字段>"):
 *   主叫(CQ 模式)   : CQ -> 等对方回答(点我方呼号+网格) -> 我方 REPORT
 *                     -> 等对方 R 报告 -> 我方 RR73 -> 等对方 73 结束
 *   应答(应答模式)   : 听到陌生 CQ -> 我方 CALL(对格) -> 等对方 REPORT
 *                     -> 我方 R 报告(回显) -> 等对方 RR73/RRR -> 我方 73 结束
 *   两个阶段之间的"我方时隙"会由 TX 任务自动重复当前内容, 引擎按
 *   我方时隙计数, 超过 max_retries 仍无进展则放弃该台。
 * ============================================================ */
#define QSO_QUEUE_LEN       24
#define QSO_RECENT_MAX      12
#define QSO_SKIP_AGE_SLOTS  48   /* 放弃后暂不重呼的时隙数(FT8≈12分钟) */
#define QSO_LOG_MAX         10

/* 引擎数据结构(类型定义见 ft8_qso.h) */
static QueueHandle_t s_qso_q = NULL;

static qso_recent_t s_recent[QSO_RECENT_MAX];
static int s_recent_n = 0;

/* 完成的 QSO 环形日志(用于串口/将来接 LCD) */
static char s_qso_log[QSO_LOG_MAX][72];
static int  s_qso_log_head = 0;
static int  s_qso_log_n = 0;

/* ---------------- 小工具 ---------------- */

/* 报告文本如 -07 / R-12 / +05 -> dB 数值 */
static int qso_db_parse(const char *s)
{
    if (s == NULL) return 0;
    if (*s == 'R') s++;
    if (*s == '\0') return 0;
    char *end = NULL;
    long v = strtol(s, &end, 10);
    return (end != s) ? (int)v : 0;
}

/* extra 是否像网格(纯字母数字 4~8 位) */
static bool str_is_gridish(const char *s)
{
    if (s == NULL) return false;
    size_t n = strlen(s);
    if (n < 4 || n > 8) return false;
    for (size_t i = 0; i < n; i++) {
        char c = s[i];
        if (!((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9')))
            return false;
    }
    return true;
}

/* 当前 UTC/本地时隙号(与 RX/TX 任务同一把尺子) */
static int64_t qso_now_slot(void)
{
    utc_try_lock_gps();
    utc_ref_t ref;
    utc_ref_get(&ref);
    int64_t now = esp_timer_get_time();
    return utc_to_us(&ref, now) / app_slot_us();
}

/* 时间戳(有 GPS 用 UTC, 否则显示 UTC--) */
static void qso_time_str(char *buf, size_t n)
{
    ft8_app_gps_time_t g;
    if (gps_cfg_get(&g) && g.valid)
        snprintf(buf, n, "%02u:%02u:%02u", g.hour, g.minute, g.second);
    else
        snprintf(buf, n, "UTC--:--:--");
}

/* ---------------- 解码消息发布(RX 任务调用) ---------------- */
static void qso_rx_publish(const ftx_message_t *msg, float freq_hz, float snr_db, int64_t slot)
{
    if (s_qso_q == NULL || msg == NULL) return;

    const ftx_message_type_t t = ftx_message_get_type(msg);
    if (t != FTX_MESSAGE_TYPE_STANDARD && t != FTX_MESSAGE_TYPE_NONSTD_CALL) return;

    qso_rx_t m;
    memset(&m, 0, sizeof(m));
    m.msg_type = t;
    m.freq_hz  = freq_hz;
    m.snr_db   = snr_db;
    m.slot     = slot;

    ftx_message_rc_t rc;
    if (t == FTX_MESSAGE_TYPE_STANDARD)
        rc = ftx_message_decode_std(msg, &s_hash_if, m.call_to, m.call_de, m.extra, m.ftypes);
    else
        rc = ftx_message_decode_nonstd(msg, &s_hash_if, m.call_to, m.call_de, m.extra, m.ftypes);
    if (rc != FTX_MESSAGE_RC_OK) return;
    if (m.call_to[0] == '\0' || m.call_de[0] == '\0') return;

    /* 队列满直接丢弃, 不阻塞 RX 解码 */
    xQueueSend(s_qso_q, &m, 0);
}

/* ---------------- 最近呼号去重 ---------------- */
static bool qso_blocked(const char *call, int64_t now_slot)
{
    for (int i = 0; i < s_recent_n; i++) {
        if (strcmp(s_recent[i].call, call) == 0) {
            if (s_recent[i].worked) return true;               /* 已完成: 本次会话不再重复通联 */
            return (now_slot - s_recent[i].slot) < QSO_SKIP_AGE_SLOTS; /* 刚放弃: 暂避一会 */
        }
    }
    return false;
}

static void qso_note(const char *call, int64_t slot, bool worked)
{
    for (int i = 0; i < s_recent_n; i++) {
        if (strcmp(s_recent[i].call, call) == 0) {
            s_recent[i].slot = slot;
            if (worked) s_recent[i].worked = true;   /* 只升级为"已完成", 不降级 */
            return;
        }
    }
    if (s_recent_n < QSO_RECENT_MAX) {
        strncpy(s_recent[s_recent_n].call, call, sizeof(s_recent[0].call) - 1);
        s_recent[s_recent_n].call[sizeof(s_recent[0].call) - 1] = '\0';
        s_recent[s_recent_n].slot = slot;
        s_recent[s_recent_n].worked = worked;
        s_recent_n++;
    } else {
        /* 满: 覆盖最老一条 */
        int old = 0;
        for (int i = 1; i < QSO_RECENT_MAX; i++)
            if (s_recent[i].slot < s_recent[old].slot) old = i;
        strncpy(s_recent[old].call, call, sizeof(s_recent[0].call) - 1);
        s_recent[old].call[sizeof(s_recent[0].call) - 1] = '\0';
        s_recent[old].slot = slot;
        s_recent[old].worked = worked;
    }
}

/* ---------------- QSO 日志 ---------------- */
static void qso_log_raw(const char *line)
{
    ESP_LOGI("ft8_qso", "%s", line);
    strncpy(s_qso_log[s_qso_log_head], line, sizeof(s_qso_log[0]) - 1);
    s_qso_log[s_qso_log_head][sizeof(s_qso_log[0]) - 1] = '\0';
    s_qso_log_head = (s_qso_log_head + 1) % QSO_LOG_MAX;
    if (s_qso_log_n < QSO_LOG_MAX) s_qso_log_n++;
}

/* ---------------- 状态机动作 ---------------- */

/* 按当前状态把"要发什么"写入 cfg(引擎接管 cfg.tx) */
static void qso_apply(qso_ctx_t *c)
{
    ft8_app_config_t *C = s_cfg_p;

    C->tx.call_to[0]   = '\0';
    C->tx.cq_modifier[0] = '\0';
    C->tx.rst_db       = 0;
    C->tx_slot_parity  = c->tx_parity & 1;

    if (c->state == QSO_ST_IDLE) {
        if (s_cfg.qso.cq_mode) {
            C->tx_enable = true;                      /* 主叫: 持续 CQ */
            C->tx.type   = FT8_APP_MSG_CQ;
        } else {
            C->tx_enable = false;                     /* 应答: 静默收听 */
            C->tx.type   = FT8_APP_MSG_CQ;
        }
        return;
    }

    C->tx_enable = true;
    strncpy(C->tx.call_to, c->peer, sizeof(C->tx.call_to) - 1);
    C->tx.call_to[sizeof(C->tx.call_to) - 1] = '\0';

    switch (c->state) {
    case QSO_ST_REPORT: C->tx.type = FT8_APP_MSG_REPORT;  C->tx.rst_db = c->my_rst;   break;
    case QSO_ST_RR73:   C->tx.type = FT8_APP_MSG_RR73;                                break;
    case QSO_ST_CALL:   C->tx.type = FT8_APP_MSG_CALL;                                break;
    case QSO_ST_RRPT:   C->tx.type = FT8_APP_MSG_R_REPORT; C->tx.rst_db = c->peer_rst; break;
    case QSO_ST_73:     C->tx.type = FT8_APP_MSG_73;                                  break;
    default: break;
    }
}

/* 由本机测得 SNR 折算我方发出的报告 dB */
static int qso_snr_to_db(float snr_db)
{
    if (!isfinite(snr_db)) return -12;
    int db = (int)lroundf(snr_db);
    if (db < -30) db = -30;
    if (db > 30)  db = 30;
    return db;
}

static const char *qso_state_name(qso_state_t s)
{
    switch (s) {
    case QSO_ST_IDLE:   return "空闲";
    case QSO_ST_REPORT: return "等R报告";
    case QSO_ST_RR73:   return "等73";
    case QSO_ST_CALL:   return "呼叫中";
    case QSO_ST_RRPT:   return "等RR73";
    case QSO_ST_73:     return "收尾";
    default:            return "?";
    }
}

static void qso_to_idle(qso_ctx_t *c)
{
    c->state     = QSO_ST_IDLE;
    c->engaged   = false;
    c->peer[0]   = '\0';
    c->attempts  = 0;
    qso_apply(c);
}

/* 通联完成: 记日志、写"已完成"去重表、回空闲 */
static void qso_complete(qso_ctx_t *c, int64_t now_slot)
{
    char t[24], line[192];
    qso_time_str(t, sizeof(t));
    snprintf(line, sizeof(line), "[%s] QSO 完成: %s %s  我发%+ddB / 收%+ddB  相位%d",
             t, c->peer, c->peer_grid[0] ? c->peer_grid : "-", c->my_rst, c->peer_rst,
             c->tx_parity);
    qso_log_raw(line);
    qso_note(c->peer, now_slot, true);
    qso_to_idle(c);
}

/* 放弃当前台: 记日志、写"暂避"去重表、回空闲 */
static void qso_give_up(qso_ctx_t *c, int64_t now_slot, int stage_attempts)
{
    char t[24], line[192];
    qso_time_str(t, sizeof(t));
    snprintf(line, sizeof(line), "[%s] 放弃 %s(阶段%s, 我方已发%u次)", t, c->peer,
             qso_state_name(c->state), (unsigned)stage_attempts);
    qso_log_raw(line);
    qso_note(c->peer, now_slot, false);
    qso_to_idle(c);
}

/* 消息语义解析(按发射方视角: call_de 恒为发射方) */
static void qso_evt_classify(const qso_rx_t *m, qso_evt_t *e)
{
    const char *our = s_cfg.callsign;
    memset(e, 0, sizeof(*e));
    e->kind = QSO_EVT_NONE;

    if (m->call_de[0] == '\0' || strchr(m->call_de, '<')) return;   /* 哈希呼号无法识别 */
    if (strcmp(m->call_de, our) == 0) return;                        /* 本机自己的发射(被监听回收) */

    strncpy(e->sender, m->call_de, sizeof(e->sender) - 1);
    e->sender[sizeof(e->sender) - 1] = '\0';
    e->parity = (int)(m->slot & 1);

    const bool to_us = (strcmp(m->call_to, our) == 0);
    const char *ex = m->extra;

    if (!to_us) {
        /* 不是点我: 只有 CQ/QRZ 呼叫才与本站相关(可应答) */
        if (strncmp(m->call_to, "CQ", 2) == 0 || strcmp(m->call_to, "QRZ") == 0) {
            e->kind = QSO_EVT_CQ;
            if (str_is_gridish(ex)) strncpy(e->grid, ex, sizeof(e->grid) - 1);
        }
        return;
    }

    switch (m->ftypes[2]) {
    case FTX_FIELD_GRID:                     /* 点我 + 网格 = 回答我的 CQ */
        e->kind = QSO_EVT_ANSWER;
        if (str_is_gridish(ex)) strncpy(e->grid, ex, sizeof(e->grid) - 1);
        break;
    case FTX_FIELD_RST:
        e->rst_db = qso_db_parse(ex);
        e->kind = (ex[0] == 'R') ? QSO_EVT_RREPORT : QSO_EVT_REPORT;
        break;
    case FTX_FIELD_TOKEN:
        if (strcmp(ex, "RRR") == 0)      e->kind = QSO_EVT_RRR;
        else if (strcmp(ex, "RR73") == 0) e->kind = QSO_EVT_RR73;
        else if (strcmp(ex, "73") == 0)  e->kind = QSO_EVT_73;
        break;
    default:
        break;
    }
}

/* 收到一条结构化消息 -> 状态推进 */
static void qso_on_rx(qso_ctx_t *c, const qso_rx_t *m)
{
    qso_evt_t ev;
    qso_evt_classify(m, &ev);
    if (ev.kind == QSO_EVT_NONE) return;

    const int64_t now_slot = qso_now_slot();
    char t[24], line[192];
    qso_time_str(t, sizeof(t));

    /* ---------------- 空闲 ---------------- */
    if (!c->engaged) {
        if (!s_cfg.qso.cq_mode) {
            /* 应答模式: 听到陌生 CQ 即应答 */
            if (ev.kind != QSO_EVT_CQ) return;
            if (qso_blocked(ev.sender, now_slot)) return;
            if (s_cfg.qso.target_callsign[0] &&
                strcmp(ev.sender, s_cfg.qso.target_callsign) != 0) return;

            c->state      = QSO_ST_CALL;
            c->engaged    = true;
            strncpy(c->peer, ev.sender, sizeof(c->peer) - 1);
            c->peer[sizeof(c->peer) - 1] = '\0';
            strncpy(c->peer_grid, ev.grid, sizeof(c->peer_grid) - 1);
            c->peer_grid[sizeof(c->peer_grid) - 1] = '\0';
            c->tx_parity  = ev.parity ^ 1;              /* 对方反相时隙发射 */
            c->attempts   = 0;
            c->last_counted = -1;
            qso_apply(c);
            snprintf(line, sizeof(line), "[%s] -> 呼叫 %s %s(改发相位%d)",
                     t, c->peer, c->peer_grid, c->tx_parity);
            qso_log_raw(line);
        } else {
            /* 主叫模式: 等"回答我 CQ"的台(点我呼号 + 网格) */
            if (ev.kind != QSO_EVT_ANSWER) return;
            if (qso_blocked(ev.sender, now_slot)) return;

            c->state      = QSO_ST_REPORT;
            c->engaged    = true;
            strncpy(c->peer, ev.sender, sizeof(c->peer) - 1);
            c->peer[sizeof(c->peer) - 1] = '\0';
            strncpy(c->peer_grid, ev.grid, sizeof(c->peer_grid) - 1);
            c->peer_grid[sizeof(c->peer_grid) - 1] = '\0';
            c->my_rst     = qso_snr_to_db(m->snr_db);
            c->tx_parity  = s_cfg.tx_slot_parity & 1;
            c->attempts   = 0;
            c->last_counted = -1;
            qso_apply(c);
            snprintf(line, sizeof(line), "[%s] %s 回答我的 CQ, 发报告 %+ddB",
                     t, c->peer, c->my_rst);
            qso_log_raw(line);
        }
        return;
    }

    /* ---------------- 进行中: 只处理来自当前 peer 的消息 ---------------- */
    if (strcmp(ev.sender, c->peer) != 0) return;

    /* 对方每次发射都重申我方相位为对方反相(抗 GPS 中途锁相导致的相位翻转) */
    {
        int want = ev.parity ^ 1;
        if (want != c->tx_parity) {
            c->tx_parity = want;
            qso_apply(c);
        }
    }

    switch (c->state) {
    case QSO_ST_REPORT:                       /* 主叫: 等对方 R 报告 / 结束 */
        if (ev.kind == QSO_EVT_REPORT || ev.kind == QSO_EVT_RREPORT) {
            c->peer_rst = ev.rst_db;
            c->state    = QSO_ST_RR73;
            c->attempts = 0;
            c->last_counted = -1;
            qso_apply(c);
            snprintf(line, sizeof(line), "[%s] 收到 %s 的%s报告 %+ddB, 发 RR73",
                     t, c->peer, (ev.kind == QSO_EVT_RREPORT) ? "R" : "", c->peer_rst);
            qso_log_raw(line);
        } else if (ev.kind == QSO_EVT_RRR || ev.kind == QSO_EVT_RR73 || ev.kind == QSO_EVT_73) {
            qso_complete(c, now_slot);
        }
        break;

    case QSO_ST_RR73:                         /* 主叫: 等对方 73 结束 */
        if (ev.kind == QSO_EVT_RRR || ev.kind == QSO_EVT_RR73 || ev.kind == QSO_EVT_73)
            qso_complete(c, now_slot);
        break;

    case QSO_ST_CALL:                         /* 应答: 等对方 REPORT */
        if (ev.kind == QSO_EVT_REPORT || ev.kind == QSO_EVT_RREPORT) {
            c->peer_rst = ev.rst_db;
            c->state    = QSO_ST_RRPT;
            c->attempts = 0;
            c->last_counted = -1;
            qso_apply(c);
            snprintf(line, sizeof(line), "[%s] 收到 %s 报告 %+ddB, 回 R 报告", t, c->peer, c->peer_rst);
            qso_log_raw(line);
        }
        break;

    case QSO_ST_RRPT:                         /* 应答: 等对方 RR73/RRR */
        if (ev.kind == QSO_EVT_RRR || ev.kind == QSO_EVT_RR73) {
            c->state    = QSO_ST_73;
            c->attempts = 0;
            c->last_counted = -1;
            qso_apply(c);
            snprintf(line, sizeof(line), "[%s] %s 收尾, 发 73", t, c->peer);
            qso_log_raw(line);
        } else if (ev.kind == QSO_EVT_73) {
            qso_complete(c, now_slot);
        }
        break;

    default:
        break;
    }
}

/* 引擎任务 */
static void ft8_qso_task(void *arg)
{
    (void)arg;
    qso_ctx_t c;
    memset(&c, 0, sizeof(c));
    c.state      = QSO_ST_IDLE;
    c.tx_parity  = s_cfg.tx_slot_parity & 1;
    c.last_counted = -1;

    if (s_cfg.qso.cq_mode) {
        qso_apply(&c);
        qso_log_raw("[QSO] 自动引擎启动: 主叫模式(自动 CQ, 完成 QSO 后自动收下一个)");
    } else {
        qso_apply(&c);
        qso_log_raw("[QSO] 自动引擎启动: 应答模式(自动应答解码到的陌生 CQ 台)");
    }

    for (;;) {
        qso_rx_t m;
        while (xQueueReceive(s_qso_q, &m, 0) == pdTRUE)
            qso_on_rx(&c, &m);

        /* 我方发射时隙节流: 计重发次数; ST_73 发完即完成 */
        if (c.engaged) {
            const int64_t slot = qso_now_slot();
            if ((slot & 1) == c.tx_parity && slot != c.last_counted) {
                c.last_counted = slot;
                c.attempts++;
                if (c.state == QSO_ST_73) {
                    qso_complete(&c, slot);
                } else if (s_cfg.qso.max_retries > 0 && c.attempts >= s_cfg.qso.max_retries) {
                    qso_give_up(&c, slot, c.attempts);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(25));
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
    cfg->gps_use_pps     = true;
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

    /* 自动 QSO 引擎默认: 关闭(保持原有手动 cfg.tx 行为), 主叫模式 */
    cfg->qso.enable           = false;      // 启用自动 QSO 引擎(启用才建队, RX 解码无队时不产生额外开销)
    cfg->qso.cq_mode          = true;       // 主叫模式: 自动 CQ, 完成 QSO 后自动收下一个
    cfg->qso.max_retries      = 4;          // 超过此次数仍无进展则放弃该台
    cfg->qso.target_callsign[0] = '\0';     // 应答模式: 只应答此呼号, 空则应答所有陌生 CQ 台
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

    s_utc_ok = false;
    if (s_cfg.utc_enable) {
        if (s_cfg.gps_utc_enable) {
            /* 选用 GPS UTC：任务会在 cfg.gps 有效且收到 PPS 后自动锁存相位并
             * 把 RX/TX 时隙栅格对齐到 UTC(:00/:15/:30/:45) */
            utc_try_lock_gps();
            ESP_LOGW(TAG, "utc_enable(gps): 等待 GPS UTC+PPS(%s)，就绪后自动对齐 UTC 栅格",
                     s_utc_ok ? "已就绪" : "未就绪");
        } else {
            s_utc_ok = (time(NULL) > SNTP_EPOCH_MIN);
            ESP_LOGW(TAG, "utc_enable(SNTP): 系统时间%s校准(仅显示用，UTC 栅格对齐请改用 GPS PPS)",
                     s_utc_ok ? "已" : "未");
        }
    }

    /* 自动 QSO 引擎(启用才建队, RX 解码无队时不产生额外开销) */
    if (s_cfg.qso.enable) {
        if (s_qso_q == NULL)
            s_qso_q = xQueueCreate(QSO_QUEUE_LEN, sizeof(qso_rx_t));
        if (xTaskCreatePinnedToCore(ft8_qso_task, "ft8_qso", 4096, NULL, 4, NULL, 0)
                != pdPASS) {
            ESP_LOGE(TAG, "ft8_qso 任务创建失败, 自动 QSO 引擎不可用");
            if (s_qso_q) { vQueueDelete(s_qso_q); s_qso_q = NULL; }
        }
    }

    xTaskCreatePinnedToCore(ft8_rx_task, "ft8_rx", STACK_RX, NULL, 6, &s_task_rx, 1);
    xTaskCreatePinnedToCore(ft8_tx_task, "ft8_tx", STACK_TX, NULL, 6, &s_task_tx, 0);
    return ESP_OK;
}
