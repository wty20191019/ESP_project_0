#pragma once
#include <stdint.h>
#include "esp_err.h"
#include "driver/gpio.h"

/* ===== WM8978 (I2C0) 引脚定义 ===== */
#define WM_I2C_SCL          GPIO_NUM_3
#define WM_I2C_SDA          GPIO_NUM_21

/* I2C0 总线频率。
 * 注意：WM8978 模块板若未外接 I2C 上拉电阻，仅靠芯片内部弱上拉时，
 * 400kHz 偶发 NACK/通信失败，请降到 100kHz；有可靠外部上拉可改回 400kHz */
#define WM8978_I2C_FREQ_HZ  (400 * 1000)

/* 单次寄存器写失败后的重试次数（应对上电初期总线毛刺/从机未就绪） */
#define WM8978_I2C_WRITE_RETRY  3

/* WM8978 I2C 从机地址（7 位，不含读写位）。
 * 与正点原子参考一致：AD0 接 GND 时固定为 0x1A */
#define WM8978_I2C_ADDR     0x1A

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 WM8978 所挂载的 I2C0 主机总线并注册从机设备（幂等）
 * @return ESP_OK 成功，否则失败
 */
esp_err_t wm8978_i2c_init(void);

/**
 * @brief 注销设备并释放 I2C0 主机总线
 * @return ESP_OK 成功，否则失败
 */
esp_err_t wm8978_i2c_deinit(void);

/**
 * @brief 向 WM8978 写入一个 9 位寄存器值
 * @note  WM8978 采用 2 字节写协议：首字节为 {reg[6:0], val[8]}，次字节为 val[7:0]。
 *        WM8978 的 I2C 接口只支持写，不支持从硬件读回，读寄存器需借助软件缓存表。
 * @param reg 寄存器地址（0~57）
 * @param val 寄存器值（9 位有效）
 * @return ESP_OK 成功（从机已应答），否则失败
 */
esp_err_t wm8978_i2c_write_reg(uint8_t reg, uint16_t val);

#ifdef __cplusplus
}
#endif
