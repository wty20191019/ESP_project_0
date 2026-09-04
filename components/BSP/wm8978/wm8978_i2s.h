#pragma once
#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/i2s_std.h"

/* ===== WM8978 (I2S0) 引脚定义 =====
 * WM8978 工作在从机模式，ESP32-S3 的 I2S0 作为主机提供 MCLK/BCLK/LRCK */
#define WM_I2S_MCLK     GPIO_NUM_12
#define WM_I2S_BCLK     GPIO_NUM_13
#define WM_I2S_LRCK     GPIO_NUM_14
#define WM_I2S_DOUT     GPIO_NUM_15   /* MCU -> CODEC (DACDAT) */
#define WM_I2S_DIN      GPIO_NUM_16   /* CODEC -> MCU (ADCDAT) */

/* I2S DMA 描述符数量与每描述符帧数（对应参考实验中的双缓冲 DMA） */
#define WM8978_I2S_DMA_DESC_NUM   6
#define WM8978_I2S_DMA_FRAME_NUM  240

/* 默认音频格式：飞利浦(I2S)标准、16 位、双声道 */
#define WM8978_I2S_DATA_BITS      I2S_DATA_BIT_WIDTH_16BIT

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 I2S0 标准模式主机，构建 TX(播放)/RX(录音) 全双工通道
 * @note  WM8978 作为 I2S 从机，本驱动负责产生 MCLK/BCLK/LRCK。
 *        声道数据为 int16 左右交替（interleave）格式。
 * @param sample_rate_hz 采样率（如 16000 / 44100 / 48000）
 * @return ESP_OK 成功，否则失败
 */
esp_err_t wm8978_i2s_init(uint32_t sample_rate_hz);

/**
 * @brief 注销 I2S0 通道并释放资源
 * @return ESP_OK 成功，否则失败
 */
esp_err_t wm8978_i2s_deinit(void);

/**
 * @brief 重新设置 I2S 采样率（如播放不同码率的 WAV 前调用）
 * @note  若通道正在运行会自动停止并重新按需启动
 * @param sample_rate_hz 新采样率
 * @return ESP_OK 成功，否则失败
 */
esp_err_t wm8978_i2s_set_sample_rate(uint32_t sample_rate_hz);

/**
 * @brief 启动 I2S0 TX（播放）通道
 * @return ESP_OK 成功，否则失败
 */
esp_err_t wm8978_i2s_start_tx(void);

/**
 * @brief 停止 I2S0 TX（播放）通道
 * @return ESP_OK 成功，否则失败
 */
esp_err_t wm8978_i2s_stop_tx(void);

/**
 * @brief 启动 I2S0 RX（录音）通道
 * @return ESP_OK 成功，否则失败
 */
esp_err_t wm8978_i2s_start_rx(void);

/**
 * @brief 停止 I2S0 RX（录音）通道
 * @return ESP_OK 成功，否则失败
 */
esp_err_t wm8978_i2s_stop_rx(void);

/**
 * @brief 向 I2S0 写音频数据（阻塞播放）
 * @param src            待发送数据缓冲
 * @param size           待发送字节数
 * @param bytes_written  实际写入字节数（可为 NULL）
 * @param timeout_ms     阻塞超时
 * @return ESP_OK 成功，ESP_ERR_TIMEOUT 超时
 */
esp_err_t wm8978_i2s_write(const void *src, size_t size, size_t *bytes_written, uint32_t timeout_ms);

/**
 * @brief 从 I2S0 读音频数据（阻塞录音）
 * @param dest         数据接收缓冲
 * @param size         期望读取字节数
 * @param bytes_read   实际读取字节数（可为 NULL）
 * @param timeout_ms   阻塞超时
 * @return ESP_OK 成功，ESP_ERR_TIMEOUT 超时
 */
esp_err_t wm8978_i2s_read(void *dest, size_t size, size_t *bytes_read, uint32_t timeout_ms);

/**
 * @brief 获取 TX 通道句柄（供上层注册事件回调等高级用法）
 */
i2s_chan_handle_t wm8978_i2s_get_tx_handle(void);

/**
 * @brief 获取 RX 通道句柄（供上层注册事件回调等高级用法）
 */
i2s_chan_handle_t wm8978_i2s_get_rx_handle(void);

#ifdef __cplusplus
}
#endif
