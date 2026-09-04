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

/** 消息构造方式：发给谁 */
typedef enum {
    FT8_APP_MSG_CQ = 0,     /*!< 呼叫 CQ：<CQ前缀> <呼号> <网格> */
    FT8_APP_MSG_CALL,       /*!< 呼叫指定台：<目标呼号> <呼号> <网格> */
} ft8_app_msg_mode_t;

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

    /* ---- 消息内容 ---- */
    ft8_app_msg_mode_t msg_mode;    /*!< CQ 呼叫 / 呼叫指定台 */
    char call_to[16];               /*!< 呼叫目标呼号(仅 CALL 模式用)，如 "BG5ABC" */
    char cq_modifier[8];            /*!< CQ 修饰：""、"DX"、"WW"、"TEST" 等(仅 CQ 模式)。
                                     *    若标准 FT8 无法表达会自动回退为纯 CQ */

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
