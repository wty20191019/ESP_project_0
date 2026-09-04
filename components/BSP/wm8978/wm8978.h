#pragma once
#include <stdint.h>

/* WM8978 寄存器驱动（逻辑层），移植自正点原子 STM32F407 参考实验，
 * I2C 底层由同目录 wm8978_i2c 模块提供（ESP32-S3 I2C0：SCL=GPIO8, SDA=GPIO9）。
 *
 * WM8978 的 I2C 接口只支持写、无法从硬件回读，因此本驱动维护一份
 * 58 个寄存器(0~57)的软件缓存表；写寄存器时同步更新缓存，
 * 读寄存器时直接返回缓存值（WM8978_Read_Reg）。 */

/* WM8978 I2C 从机地址（7 位，不含读写位），AD0 接地时为 0x1A */
#define WM8978_ADDR                 0x1A

/* EQ1 截止频率选择（R18） */
#define EQ1_80Hz                    0X00
#define EQ1_105Hz                   0X01
#define EQ1_135Hz                   0X02
#define EQ1_175Hz                   0X03

/* EQ2 截止频率选择（R19） */
#define EQ2_230Hz                   0X00
#define EQ2_300Hz                   0X01
#define EQ2_385Hz                   0X02
#define EQ2_500Hz                   0X03

/* EQ3 截止频率选择（R20） */
#define EQ3_650Hz                   0X00
#define EQ3_850Hz                   0X01
#define EQ3_1100Hz                  0X02
#define EQ3_14000Hz                 0X03

/* EQ4 截止频率选择（R21） */
#define EQ4_1800Hz                  0X00
#define EQ4_2400Hz                  0X01
#define EQ4_3200Hz                  0X02
#define EQ4_4100Hz                  0X03

/* EQ5 截止频率选择（R22） */
#define EQ5_5300Hz                  0X00
#define EQ5_6900Hz                  0X01
#define EQ5_9000Hz                  0X02
#define EQ5_11700Hz                 0X03

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief WM8978 初始化
 * @note  内部会一并初始化 I2C 底层；上电复位 WM8978 并写入常用默认配置。
 *        本函数不配置 I2S 时钟（由 wm8978_i2s 模块负责）。
 * @return 0：初始化成功；非 0：初始化失败
 */
uint8_t WM8978_Init(void);

/**
 * @brief DAC/ADC 通路使能配置
 * @param dacen DAC 使能(1)/关闭(0)
 * @param adcen ADC 使能(1)/关闭(0)
 */
void WM8978_ADDA_Cfg(uint8_t dacen, uint8_t adcen);

/**
 * @brief 输入通道配置（MIC / Line In / AUX）
 * @param micen     MIC 使能(1)/关闭(0)
 * @param lineinen  Line In 使能(1)/关闭(0)
 * @param auxen     AUX 使能(1)/关闭(0)
 */
void WM8978_Input_Cfg(uint8_t micen, uint8_t lineinen, uint8_t auxen);

/**
 * @brief 输出通道配置
 * @param dacen DAC 输出(功放前级)使能(1)/关闭(0)
 * @param bpsen Bypass 直通输出(监听 MIC/LineIn/AUX)使能(1)/关闭(0)
 */
void WM8978_Output_Cfg(uint8_t dacen, uint8_t bpsen);

/**
 * @brief MIC 输入增益（含 BOOST 20dB，MIC-->ADC 输入部分增益）
 * @param gain 0~63，对应 -12dB ~ +35.25dB，0.75dB/步
 */
void WM8978_MIC_Gain(uint8_t gain);

/**
 * @brief LINE IN 输入增益（L2/R2-->ADC 输入部分增益）
 * @param gain 0~7：0 表示通路截止；1~7 对应 -12dB ~ +6dB，3dB/步
 */
void WM8978_LINEIN_Gain(uint8_t gain);

/**
 * @brief AUX 输入增益（AUXR/AUXL-->ADC 输入部分增益）
 * @param gain 0~7：0 表示通路截止；1~7 对应 -12dB ~ +6dB，3dB/步
 */
void WM8978_AUX_Gain(uint8_t gain);

/**
 * @brief 写 WM8978 寄存器
 * @param reg 寄存器地址（0~57）
 * @param val 寄存器值（9 位有效）
 * @return 0：成功；非 0：失败
 */
uint8_t WM8978_Write_Reg(uint8_t reg, uint16_t val);

/**
 * @brief 读 WM8978 寄存器（返回软件缓存表中的值）
 * @param reg 寄存器地址（0~57）
 * @return 寄存器值
 */
uint16_t WM8978_Read_Reg(uint8_t reg);

/**
 * @brief 耳机(HP)音量设置
 * @param voll 左声道音量(0~63，0 为静音)
 * @param volr 右声道音量(0~63，0 为静音)
 */
void WM8978_HPvol_Set(uint8_t voll, uint8_t volr);

/**
 * @brief 喇叭(SPK)音量设置
 * @param volx 音量(0~63，0 为静音)
 */
void WM8978_SPKvol_Set(uint8_t volx);

/**
 * @brief 设置 I2S 音频格式（需与 wm8978_i2s 模块保持一致）
 * @param fmt 0：LSB(右对齐)；1：MSB(左对齐)；2：飞利浦标准 I2S；3：PCM/DSP
 * @param len 0：16 位；1：20 位；2：24 位；3：32 位
 */
void WM8978_I2S_Cfg(uint8_t fmt, uint8_t len);

/**
 * @brief 设置 3D 音效强度
 * @param depth 0~15（0 关闭，15 最强）
 */
void WM8978_3D_Set(uint8_t depth);

/**
 * @brief 设置 EQ/3D 生效方向
 * @param dir 0：作用于 ADC 通路；1：作用于 DAC 通路（默认）
 */
void WM8978_EQ_3D_Dir(uint8_t dir);

/**
 * @brief 设置 EQ1 频段参数（中心频率 80/105/135/175Hz）
 * @param cfreq 截止频率选择，取 EQ1_xxx
 * @param gain  增益 0~24，对应 -12dB ~ +12dB
 */
void WM8978_EQ1_Set(uint8_t cfreq, uint8_t gain);

/**
 * @brief 设置 EQ2 频段参数（中心频率 230/300/385/500Hz）
 */
void WM8978_EQ2_Set(uint8_t cfreq, uint8_t gain);

/**
 * @brief 设置 EQ3 频段参数（中心频率 650/850/1100/1400Hz）
 */
void WM8978_EQ3_Set(uint8_t cfreq, uint8_t gain);

/**
 * @brief 设置 EQ4 频段参数（中心频率 1800/2400/3200/4100Hz）
 */
void WM8978_EQ4_Set(uint8_t cfreq, uint8_t gain);

/**
 * @brief 设置 EQ5 频段参数（中心频率 5300/6900/9000/11700Hz）
 */
void WM8978_EQ5_Set(uint8_t cfreq, uint8_t gain);

#ifdef __cplusplus
}
#endif
