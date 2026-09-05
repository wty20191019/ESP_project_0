#ifndef __LCD_INIT_H
#define __LCD_INIT_H

#include <stdint.h>
#include <stddef.h>
#include "driver/gpio.h"

/* ============================================================
 * ST7735 1.8 寸 TFT(128x160) 底层驱动配置
 * 参考: STM32F103 硬件 SPI+DMA 例程 HARDWARE/LCD/lcd_init.h
 * 适配: ESP32-S3, 硬件 SPI2 + DMA(spi_master)
 * ============================================================ */

/* ---------- 显示方向 (与参考例程一致) ----------
 * USE_HORIZONTAL: 0/1 竖屏(128x160), 2/3 横屏(160x128) */
#ifndef USE_HORIZONTAL
#define USE_HORIZONTAL 1
#endif

#if (USE_HORIZONTAL == 0) || (USE_HORIZONTAL == 1)
#define LCD_W   128
#define LCD_H   160
#else
#define LCD_W   160
#define LCD_H   128
#endif

/* ---------- 引脚定义(硬件接线) ----------
 * 默认按如下接线(可按需在编译前覆盖):
 *   LCD_SCK  -> GPIO10  (SPI2 SCLK) //SCL
 *   LCD_MOSI -> GPIO11  (SPI2 MOSI) //SDA
 *   LCD_CS   -> GPIO42  (片选, 低有效)
 *   LCD_DC   -> GPIO4   (数据/命令, 高=数据 低=命令)
 *   LCD_RST  -> GPIO5   (复位, 低有效)//RES
 *   LCD_BLK  -> GPIO6   (背光, 高电平点亮) */
#ifndef LCD_SCK
#define LCD_SCK     GPIO_NUM_10
#endif
#ifndef LCD_MOSI
#define LCD_MOSI    GPIO_NUM_11
#endif
#ifndef LCD_CS
#define LCD_CS      GPIO_NUM_42
#endif
#ifndef LCD_DC
#define LCD_DC      GPIO_NUM_4
#endif
#ifndef LCD_RST
#define LCD_RST     GPIO_NUM_5
#endif
#ifndef LCD_BLK
#define LCD_BLK     GPIO_NUM_6
#endif

/* SPI 总线时钟(Hz)。ST7735 标称 ~15MHz, 该模块实测可跑更高, 默认 40MHz */
#ifndef LCD_SPI_CLK_HZ
#define LCD_SPI_CLK_HZ  (40 * 1000 * 1000)
#endif

/* ---------- 底层函数(由 lcd_init.c 实现) ---------- */
void LCD_GPIO_Init(void);                 /* 初始化控制 GPIO: DC/CS/RST/BLK */
void LCD_SPI_Init(void);                  /* 初始化硬件 SPI2(带 DMA) */
void LCD_WR_REG(uint8_t dat);             /* 写 1 字节命令 (DC=0) */
void LCD_WR_DATA8(uint8_t dat);           /* 写 1 字节数据 (DC=1) */
void LCD_WR_DATA(uint16_t dat);           /* 写 16 位颜色/数据, 高字节在前 */
void LCD_Address_Set(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void LCD_Push(const uint8_t *buf, size_t len);  /* 连续推送像素数据, 内部 DMA 分块, 全程拉低 CS */
void LCD_Backlight(uint8_t on);           /* 背光控制 */
void LCD_Init(void);                      /* 上电初始化 */

#endif
