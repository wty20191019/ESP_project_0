#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "ft8/constants.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * FT8/FT4 应用控制器（单任务：连续解码 + 按 UTC 时隙自动收发）
 *
 * 依赖底层组件：
 *   - components/ft8_lib : 编解码/瀑布(monitor)
 *   - components/BSP/wm8978 : I2S/I2C 音频通路(12kHz 全双工)
 *
 * ── FT8 真实时序(两层) ────────────────────────────────
 *   宏观层：每 15s 一个时隙，严格对齐 UTC 分钟边界(:00/:15/:30/:45)。
 *           台站按"奇偶时隙"自动交替：本台只在被选中(偶数或奇数)的
 *           时隙发射，其余时隙静默接收(与对端自动错开)。
 *   微观层：一个时隙前 12.64s 为有效波形(79 符号×0.16s)，
 *           后约 2.36s 为静默间隙，用于解码与 T/R 切换。
 *   波形：8-GFSK 调制(高斯成形)，tone 间距 6.25Hz，占用约 50Hz 带宽。
 *
 *   FT4 同理，只是时隙 7.5s、符号 0.048s(105 符号)。
 *
 *   要求：utc_enable=1 时系统时钟需已由 SNTP 校准(误差<1s)，否则自动
 *   回退到以上电时刻为起点的本地栅格(仅用于无网测试)。
 * ============================================================ */

/**
 * 发射消息类型：标准一次通联的 6 类内容。
 * 文本一律以本机(呼号=callsign)为发送方视角拼接，格式为：
 *   CQ 类    "CQ [修饰] <本机> <本机网格>"
 *   其它类   "<目标呼号> <本机> <第三字段>"
 */
typedef enum {
    FT8_APP_MSG_CQ = 0,     /*!< 第1类 CQ 呼叫：CQ [修饰] <本机呼号> <本机网格> */
    FT8_APP_MSG_CALL,       /*!< 第2类 应答呼叫：<目标呼号> <本机呼号> <本机网格> */
    FT8_APP_MSG_REPORT,     /*!< 第3类 信号报告：<目标呼号> <本机呼号> <±dB>，如 -10 */
    FT8_APP_MSG_R_REPORT,   /*!< 第4类 R 回报告：<目标呼号> <本机呼号> R<±dB>，如 R-12 */
    FT8_APP_MSG_RRR,        /*!< 第5类 RRR：<目标呼号> <本机呼号> RRR */
    FT8_APP_MSG_RR73,       /*!< 第5类 RR73(一步结束通联)：<目标呼号> <本机呼号> RR73 */
    FT8_APP_MSG_73,         /*!< 第6类 73：<目标呼号> <本机呼号> 73 */
} ft8_app_msg_type_t;

/** 单次发射的消息参数(TX 任务每个本台时隙前重新读取，可运行中热切换) */
typedef struct {
    ft8_app_msg_type_t type;    /*!< 第几类标准消息(见上) */
    char call_to[16];           /*!< 目标呼号(类型 2~6 用)，如 "BG5ABC"；CQ 类忽略 */
    char cq_modifier[8];        /*!< CQ 修饰(仅 CQ 类用)：""、"DX"、"WW"、"TEST" 等，
                                 *    标准 FT8 无法表达时会自动回退为纯 CQ */
    int  rst_db;                /*!< 信号报告 dB(仅 REPORT / R_REPORT 类用)，
                                 *    如 -12；本库可编码范围约 ±30(再大与 RRR/RR73/73
                                 *    特殊值冲突)，超出会被截断并告警 */
} ft8_app_tx_msg_t;

/** WM8978 编解码器初始化参数(仅 ft8_app_start 启动时应用一次) */
typedef struct {
    uint8_t dac_en;      /*!< DAC 通路使能 1/0，WM8978_ADDA_Cfg(dac_en, adc_en) */
    uint8_t adc_en;      /*!< ADC 通路使能 1/0 */
    uint8_t mic_en;      /*!< MIC 输入使能 1/0，WM8978_Input_Cfg(mic_en, linein_en, aux_en) */
    uint8_t linein_en;   /*!< Line In 输入使能 1/0 */
    uint8_t aux_en;      /*!< AUX 输入使能 1/0 */
    uint8_t mic_gain;    /*!< MIC 增益 0~63(-12~+35.25dB，0.75dB/步) */
    uint8_t out_dac;     /*!< DAC 输出使能(功放前级) 1/0，WM8978_Output_Cfg(out_dac, out_bypass) */
    uint8_t out_bypass;  /*!< Bypass 直通输出使能 1/0 */
    uint8_t i2s_fmt;     /*!< I2S 格式 0~3(2=飞利浦标准 I2S)，WM8978_I2S_Cfg(i2s_fmt, i2s_len) */
    uint8_t i2s_len;     /*!< I2S 位宽 0~3(0=16bit) */
    uint8_t hp_vol_l;    /*!< 耳机左声道音量 0~63(0 静音)，WM8978_HPvol_Set(l, r) */
    uint8_t hp_vol_r;    /*!< 耳机右声道音量 0~63(0 静音) */
    uint8_t spk_vol;     /*!< 喇叭音量 0~63(0 静音)，WM8978_SPKvol_Set */
} ft8_app_codec_cfg_t;

/** FT8/FT4 应用配置结构体 */
typedef struct {
    /* ---- 协议与开关 ---- */
    ftx_protocol_t protocol;    /*!< FTX_PROTOCOL_FT8(15s 时隙) / FTX_PROTOCOL_FT4(7.5s) */
    bool tx_enable;             /*!< 是否参与发射(在选中时隙内) */
    bool rx_enable;             /*!< 是否持续接收解码 */

    /* ---- 时间(宏观层) ---- */
    bool utc_enable;            /*!< 用系统 UTC 时间对齐 15s/7.5s 栅格(需先 SNTP 校时)；
                                 *    为 false 或时间未校准时退回本地栅格 */
    int  tx_slot_parity;        /*!< 0=在偶数时隙发射(WSJT-X "even" 默认)；
                                 *    1=在奇数时隙发射，对端自动落在另一奇偶 */
    uint32_t tx_delay_ms;       /*!< 本台时隙内再延时多少 ms 开始发射(0~时隙长-消息长) */

    /* ---- 电台身份 ---- */
    char callsign[16];          /*!< 本机呼号，如 "BG7ABC" */
    char grid[8];               /*!< 本机网格，如 "JO70" */

    /* ---- 消息内容(TX 任务每时隙前重新读取；运行中直接改 cfg.tx.* 即可热切换) ---- */
    ft8_app_tx_msg_t tx;

    /* ---- WM8978 编解码器初始化(仅启动时使用，运行中不建议改) ---- */
    ft8_app_codec_cfg_t codec;

    /* ---- 音频(发射，微观层) ---- */
    float audio_freq_hz;        /*!< tone0 中心频率 Hz(常用 1200/1500)，8-GFSK 高斯成形 */
    float audio_level;          /*!< 发射幅度 0~1，防削波建议 ≤0.9 */

    /* ---- 解码(接收) ---- */
    float rx_f_min;             /*!< 解码频率下限 Hz */
    float rx_f_max;             /*!< 解码频率上限 Hz */
    int rx_time_osr;            /*!< 时间细分(≥1) */
    int rx_freq_osr;            /*!< 频率细分(≥1) */
    int max_candidates;         /*!< 每时隙候选数 */
    int ldpc_iterations;        /*!< LDPC 最大迭代次数 */
    uint32_t rx_parse_ms;       /*!< 每个时隙结束前提前多少 ms 停止接收并开始解析
                                 *    (解析时间预算，默认 1500)；应小于时隙静默段时长 */

    uint8_t _reserved[8];
} ft8_app_config_t;

/** 填充默认配置(FT8 / BG7ABC / JO70 / 纯 CQ / UTC 偶时隙 / 0 延时 / WM8978 常用参数) */
void ft8_app_config_default(ft8_app_config_t *cfg);

/**
 * @brief 启动 FT8/FT4 应用任务
 *
 * 单任务内完成：WM8978+I2S 音频初始化 -> 逐符号采集喂瀑布 -> 每时隙解码一次；
 * 同时在"本台奇偶时隙"内按 发射延时 播放预生成的 8-GFSK 波形(其余补零静默)。
 *
 * @note 配置采用"引用"而非拷贝：模块会持续读取 cfg 中的消息内容(cfg.tx /
 *       callsign / grid 等)来决定下一时隙发什么。因此 cfg 必须在整个运行期间
 *       保持有效(建议定义为全局或 static 变量)；想切换发射消息时直接修改
 *       cfg.tx.type / cfg.tx.call_to / cfg.tx.rst_db 即可，下一本台时隙生效。
 *       其余如时隙/协议/音频等参数以启动时刻为准。
 *
 * @note 只应调用一次；重复调用将被忽略并返回 ESP_OK
 * @param cfg 配置(不会被复制，需保持有效)
 * @return ESP_OK 成功
 */
esp_err_t ft8_app_start(const ft8_app_config_t *cfg);

#ifdef __cplusplus
}
#endif
