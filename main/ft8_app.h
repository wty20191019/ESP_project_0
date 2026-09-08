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

    /* ---- 消息内容(可运行中热切换，见 ft8_app_tx_msg_set) ---- */
    ft8_app_tx_msg_t tx;

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

    uint8_t _reserved[8];
} ft8_app_config_t;

/** 填充默认配置(FT8 / BG7ABC / JO70 / 纯 CQ / UTC 偶时隙 / 0 延时) */
void ft8_app_config_default(ft8_app_config_t *cfg);

/**
 * @brief 运行中热切换 TX 发射消息(线程安全)
 *
 * 在任一其它任务里修改 type/call_to/rst_db 等后调用，TX 任务会在下一个
 * 本台时隙前重新编码并生成波形，无需重启 FT8 应用。
 *
 * @param tx 新的消息参数(会被复制)
 */
void ft8_app_tx_msg_set(const ft8_app_tx_msg_t *tx);

/**
 * @brief 启动 FT8/FT4 应用任务(配置会被复制)
 *
 * 单任务内完成：I2S/WM8978 音频初始化 -> 逐符号采集喂瀑布 -> 每时隙解码一次；
 * 同时在"本台奇偶时隙"内按 发射延时 播放预生成的 8-GFSK 波形(其余补零静默)。
 *
 * @note 只应调用一次；重复调用将被忽略并返回 ESP_OK
 * @param cfg 配置(会被复制，可在栈上使用)
 * @return ESP_OK 成功
 */
esp_err_t ft8_app_start(const ft8_app_config_t *cfg);

#ifdef __cplusplus
}
#endif
