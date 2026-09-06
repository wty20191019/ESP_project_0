#ifndef __LCD_H
#define __LCD_H

#include <stdint.h>
#include "lcd_init.h"

/* ============================================================
 * ST7735 1.8 寸 TFT(128x160) 上层绘图/字符 API
 * 参考: STM32F103 例程 HARDWARE/LCD/lcd.h
 * 说明: 绘图函数先写离屏整帧缓冲, 再经 LCD_Flush() 一次性 DMA 上屏,
 *       避免逐行直推造成的"扫描/闪烁"。
 * ============================================================ */

/* ------------------------- 基础图形 ------------------------- */
void LCD_Fill(uint16_t xsta, uint16_t ysta, uint16_t xend, uint16_t yend, uint16_t color); /* 区域填充(不含右/下边界) */
void LCD_Clear(uint16_t color);                                                            /* 清屏(写缓冲) */
void LCD_Flush(void);                                                                      /* 整帧一次推送显示 */
void LCD_DrawPoint(uint16_t x, uint16_t y, uint16_t color);                                 /* 画点 */
void LCD_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);      /* 画线 */
void LCD_DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color); /* 空心矩形 */
void Draw_Circle(uint16_t x0, uint16_t y0, uint8_t r, uint16_t color);                      /* 空心圆 */

/* ------------------------- 字符显示 ------------------------- */
/* sizey 可选: 12 / 16 / 24 / 32 (字形分别为 6x12 / 8x16 / 12x24 / 16x32)
 * mode: 0=带底色(实心)  1=透明叠加 */
void LCD_ShowChar(uint16_t x, uint16_t y, uint8_t num, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode);
void LCD_ShowString(uint16_t x, uint16_t y, const uint8_t *p, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode);
uint32_t mypow(uint8_t m, uint8_t n);
void LCD_ShowIntNum(uint16_t x, uint16_t y, uint16_t num, uint8_t len, uint16_t fc, uint16_t bc, uint8_t sizey);
void LCD_ShowFloatNum1(uint16_t x, uint16_t y, float num, uint8_t len, uint16_t fc, uint16_t bc, uint8_t sizey);

/* ------------------------- 图片显示 ------------------------- */
/* pic[]: RGB565 数组(高字节在前), 大小 = length*width*2 */
void LCD_ShowPicture(uint16_t x, uint16_t y, uint16_t length, uint16_t width, const uint8_t pic[]);

/* ------------------------- 常用颜色 ------------------------- */
//RGB565 格式（16位：R[15:11] + G[10:5] + B[4:0]）

#define WHITE       0xFFFF    // 白色
#define BLACK       0x0000    // 黑色
#define BLUE        0x001F    // 蓝色
#define BRED        0xF81F    // 紫红色（红+蓝）
#define GRED        0xFFE0    // 黄色（红+绿）
#define GBLUE       0x07FF    // 青色（绿+蓝）
#define RED         0xF800    // 红色
#define MAGENTA     0xF81F    // 品红（同 BRED）
#define GREEN       0x07E0    // 绿色
#define CYAN        0x7FFF    // 青色（浅蓝绿）
#define YELLOW      0xFFE0    // 黄色（同 GRED）
#define BROWN       0xBC40    // 棕色
#define BRRED       0xFC07    // 亮红色
#define GRAY        0x8430    // 灰色
#define DARKBLUE    0x01CF    // 深蓝色
#define LIGHTBLUE   0x7D7C    // 浅蓝色
#define GRAYBLUE    0x5458    // 灰蓝色
#define LIGHTGREEN  0x841F    // 浅绿色
#define LGRAY       0xC618    // 浅灰色
#define LGRAYBLUE   0xA651    // 浅灰蓝色
#define LBBLUE      0x2B12    // 淡蓝紫色

#endif
