#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "ft8/constants.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * FT8/FT4 应用控制器（单任务：连续解码 + 每个周期定时发射）
 *
 * 依赖底层组件：
 *   - components/ft8_lib : 编解码/瀑布(monitor)
 *   - components/BSP/wm8978 : I2S/I2C 音频通路(12kHz 全双工)
 *
 * 调用方只需填好 ft8_app_config_t 并调用 ft8_app_start()。
 * 音频通路(W-M8978/I2S0)的初始化由模块内部完成，且为幂等。
 * ============================================================ */

/** 消息构造方式：决定发给谁 */
typedef enum {
    FT8_APP_MSG_CQ = 0,     /*!< 呼叫 CQ：<CQ前缀> <呼号> <网格> */
    FT8_APP_MSG_CALL,       /*!< 呼叫指定台：<目标呼号> <呼号> <网格> */
} ft8_app_msg_mode_t;

/** FT8/FT4 应用配置结构体 */
typedef struct {
    /* ---- 协议与开关 ---- */
    ftx_protocol_t protocol;    /*!< FTX_PROTOCOL_FT8(15s) / FTX_PROTOCOL_FT4(7.5s) */
    bool tx_enable;             /*!< 是否周期发射 */
    bool rx_enable;             /*!< 是否持续接收解码 */

    /* ---- 电台身份 ---- */
    char callsign[16];          /*!< 本机呼号，如 "BG7ABC" */
    char grid[8];               /*!< 本机网格，如 "JO70" */

    /* ---- 消息内容 ---- */
    ft8_app_msg_mode_t msg_mode;    /*!< CQ 呼叫 / 呼叫指定台 */
    char call_to[16];               /*!< 呼叫目标呼号(仅 CALL 模式用)，如 "BG5ABC" */
    char cq_modifier[8];            /*!< CQ 修饰：""、"DX"、"WW"、"TEST" 等(仅 CQ 模式)。
                                     *   注: 若标准 FT8 编码无法表达该修饰则自动回退为纯 CQ */
    /* ---- 时间调度 ---- */
    uint32_t tx_delay_ms;       /*!< 槽开始(含时间偏移)后再延时多少 ms 开始发射。
                                 *    需保证 延时+消息时长 < 槽长，否则该槽跳过发射 */
    int32_t time_offset_ms;     /*!< 时间栅格整体偏移，用于把 15s/7.5s 栅格对齐到
                                 *    UTC(±15000 内)，单位 ms */

    /* ---- 音频(发射) ---- */
    float audio_freq_hz;        /*!< tone0 音频中心频率 Hz，常用 1200/1500 */
    float audio_level;          /*!< 发射音频幅度 0.0~1.0，防削波建议 ≤0.9 */

    /* ---- 解码(接收) ---- */
    float rx_f_min;             /*!< 解码频率下限 Hz(如 0) */
    float rx_f_max;             /*!< 解码频率上限 Hz(如 4000) */
    int rx_time_osr;            /*!< 时间细分(≥1，越大越耗内存) */
    int rx_freq_osr;            /*!< 频率细分(≥1) */
    int max_candidates;         /*!< 每周期候选数 */
    int ldpc_iterations;        /*!< LDPC 最大迭代次数 */

    uint8_t _reserved[8];       /*!< 预留 */
} ft8_app_config_t;

/** 填充一份默认配置(FT8 / BG7ABC / JO70 / 纯 CQ，收发全开) */
void ft8_app_config_default(ft8_app_config_t *cfg);

/**
 * @brief 启动 FT8/FT4 应用任务(配置会被复制)
 *
 * 单任务内完成：I2S/WM8978 音频初始化 -> 周期采样喂解码瀑布 ->
 * 每个协议槽(FT8 15s / FT4 7.5s)解码一次；同时按 时间偏移+发射延时 在槽内发射。
 *
 * @note 只应调用一次；重复调用将被忽略并返回 ESP_OK
 * @param cfg 配置(会被复制，可在栈上使用)
 * @return ESP_OK 成功
 */
esp_err_t ft8_app_start(const ft8_app_config_t *cfg);

#ifdef __cplusplus
}
#endif
