#include "lcd.h"
#include "lcdfont.h"

/* ============================================================
 * ST7735 1.8 寸 TFT(128x160) 上层绘图/字符 API
 * 参考: STM32F103 例程 HARDWARE/LCD/lcd.c
 *
 * 说明:
 *  - 内部实现把连续的像素颜色先攒入 RAM 缓冲, 满了再通过 LCD_Push
 *    一次性 DMA 送给屏(仍在上一命令设置的窗口内, GRAM 地址自动递增)。
 *  - 除特殊说明外, API 名称与参考例程完全一致, 便于移植。
 * ============================================================ */

/* ----------------------- 像素缓冲 ----------------------- */
#define PIX_BUF_SIZE 1024
static uint8_t  s_pix[PIX_BUF_SIZE];
static uint16_t s_pix_len = 0;

/* 把缓冲内像素一次性刷给屏幕 */
static void pix_flush(void)
{
    if (s_pix_len != 0) {
        LCD_Push(s_pix, s_pix_len);
        s_pix_len = 0;
    }
}

/* 追加一个 16bit 颜色(高字节在前)到缓冲, 满则先刷 */
static void pix_emit(uint16_t color)
{
    if (s_pix_len + 2 > PIX_BUF_SIZE) {
        pix_flush();
    }
    s_pix[s_pix_len++] = color >> 8;
    s_pix[s_pix_len++] = color & 0xFF;
}

/* 最大列数对应的单行缓冲(横屏最多 160 列) */
static uint8_t s_row[160 * 2];

/* ----------------------- 基础图形 ----------------------- */

/* 区域填充: 填充 [xsta, xend) x [ysta, yend), 不包含 xend/yend 边界 */
void LCD_Fill(uint16_t xsta, uint16_t ysta, uint16_t xend, uint16_t yend, uint16_t color)
{
    uint16_t w = xend - xsta;
    uint16_t h = yend - ysta;
    uint16_t i, j;

    if (w == 0 || h == 0) {
        return;
    }
    LCD_Address_Set(xsta, ysta, xend - 1, yend - 1);   /* 含边界坐标 */

    for (i = 0; i < w; i++) {                          /* 拼一整行颜色 */
        s_row[i * 2]     = color >> 8;
        s_row[i * 2 + 1] = color & 0xFF;
    }
    for (j = 0; j < h; j++) {                          /* 逐行 DMA */
        LCD_Push(s_row, (size_t)w * 2);
    }
}

/* 整屏清屏 */
void LCD_Clear(uint16_t color)
{
    LCD_Fill(0, 0, LCD_W, LCD_H, color);
}

/* 画一个点 */
void LCD_DrawPoint(uint16_t x, uint16_t y, uint16_t color)
{
    LCD_Address_Set(x, y, x, y);
    pix_emit(color);
    pix_flush();
}

/* 画线 (Bresenham, 水平/垂直线已优化为 DMA 填充) */
void LCD_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    uint16_t t;
    int xerr = 0, yerr = 0, delta_x, delta_y, distance;
    int incx, incy, uRow, uCol;

    /* 水平 / 垂直线用整块填充, 更快 */
    if (y1 == y2) {                                   /* 水平 */
        if (x1 > x2) { t = x1; x1 = x2; x2 = t; }
        LCD_Fill(x1, y1, x2 + 1, y1 + 1, color);
        return;
    }
    if (x1 == x2) {                                   /* 垂直 */
        if (y1 > y2) { t = y1; y1 = y2; y2 = t; }
        LCD_Fill(x1, y1, x1 + 1, y2 + 1, color);
        return;
    }

    delta_x = x2 - x1;
    delta_y = y2 - y1;
    uRow = x1;
    uCol = y1;
    if (delta_x > 0) incx = 1;
    else if (delta_x == 0) incx = 0;
    else { incx = -1; delta_x = -delta_x; }

    if (delta_y > 0) incy = 1;
    else if (delta_y == 0) incy = 0;
    else { incy = -1; delta_y = -delta_y; }

    if (delta_x > delta_y) distance = delta_x;
    else distance = delta_y;

    for (t = 0; t < (uint16_t)distance + 1; t++) {
        LCD_DrawPoint(uRow, uCol, color);
        xerr += delta_x;
        yerr += delta_y;
        if (xerr > distance) { xerr -= distance; uRow += incx; }
        if (yerr > distance) { yerr -= distance; uCol += incy; }
    }
}

/* 空心矩形 */
void LCD_DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    LCD_DrawLine(x1, y1, x2, y1, color);
    LCD_DrawLine(x1, y1, x1, y2, color);
    LCD_DrawLine(x1, y2, x2, y2, color);
    LCD_DrawLine(x2, y1, x2, y2, color);
}

/* 空心圆 (Bresenham) */
void Draw_Circle(uint16_t x0, uint16_t y0, uint8_t r, uint16_t color)
{
    int a = 0, b = r;

    while (a <= b) {
        LCD_DrawPoint(x0 - b, y0 - a, color);
        LCD_DrawPoint(x0 + b, y0 - a, color);
        LCD_DrawPoint(x0 - a, y0 + b, color);
        LCD_DrawPoint(x0 - a, y0 - b, color);
        LCD_DrawPoint(x0 + b, y0 + a, color);
        LCD_DrawPoint(x0 + a, y0 - b, color);
        LCD_DrawPoint(x0 + a, y0 + b, color);
        LCD_DrawPoint(x0 - b, y0 + a, color);
        a++;
        if ((a * a + b * b) > (r * r)) {   /* 判断要描的点是否超圆 */
            b--;
        }
    }
}

/* ----------------------- 字符显示 ----------------------- */

/* 显示一个 ASCII 字符
 * sizey 可选 12/16/24/32; mode: 0=带底色, 1=透明叠加 */
void LCD_ShowChar(uint16_t x, uint16_t y, uint8_t num, uint16_t fc, uint16_t bc,
                  uint8_t sizey, uint8_t mode)
{
    uint8_t temp, sizex, t, m = 0;
    uint16_t i, typeface_num;   /* 一个字符占用的字节数 */
    uint16_t x0 = x;

    sizex = sizey / 2;
    typeface_num = (sizex / 8 + ((sizex % 8) ? 1 : 0)) * sizey;
    num = num - ' ';            /* 得到偏移后的值 */

    LCD_Address_Set(x, y, x + sizex - 1, y + sizey - 1);   /* 设置光标位置 */

    for (i = 0; i < typeface_num; i++) {
        if (sizey == 12)       temp = ascii_1206[num][i];
        else if (sizey == 16)  temp = ascii_1608[num][i];
        else if (sizey == 24)  temp = ascii_2412[num][i];
        else if (sizey == 32)  temp = ascii_3216[num][i];
        else return;

        for (t = 0; t < 8; t++) {
            if (mode == 0) {                    /* 实心(带底色)模式 */
                if (temp & (0x01 << t)) pix_emit(fc);
                else                     pix_emit(bc);
                m++;
                if (m % sizex == 0) {
                    m = 0;
                    break;
                }
            } else {                            /* 透明叠加模式 */
                if (temp & (0x01 << t)) LCD_DrawPoint(x, y, fc);
                x++;
                if ((x - x0) == sizex) {
                    x = x0;
                    y++;
                    break;
                }
            }
        }
    }
    pix_flush();
}

/* 显示字符串 */
void LCD_ShowString(uint16_t x, uint16_t y, const uint8_t *p, uint16_t fc, uint16_t bc,
                    uint8_t sizey, uint8_t mode)
{
    while (*p != '\0') {
        LCD_ShowChar(x, y, *p, fc, bc, sizey, mode);
        x += sizey / 2;
        p++;
    }
}

/* 乘方 */
uint32_t mypow(uint8_t m, uint8_t n)
{
    uint32_t result = 1;
    while (n--) result *= m;
    return result;
}

/* 显示整数, len 为总位数(前导空格补齐), 例 LCD_ShowIntNum(0,0,108,5,...,16) */
void LCD_ShowIntNum(uint16_t x, uint16_t y, uint16_t num, uint8_t len, uint16_t fc,
                    uint16_t bc, uint8_t sizey)
{
    uint8_t t, temp;
    uint8_t enshow = 0;
    uint8_t sizex = sizey / 2;

    for (t = 0; t < len; t++) {
        temp = (num / mypow(10, len - t - 1)) % 10;
        if (enshow == 0 && t < (len - 1)) {
            if (temp == 0) {
                LCD_ShowChar(x + t * sizex, y, ' ', fc, bc, sizey, 0);
                continue;
            }
            enshow = 1;
        }
        LCD_ShowChar(x + t * sizex, y, temp + 48, fc, bc, sizey, 0);
    }
}

/* 显示带 1 位小数的浮点数, len 为总位数(含小数点和小数位) */
void LCD_ShowFloatNum1(uint16_t x, uint16_t y, float num, uint8_t len, uint16_t fc,
                       uint16_t bc, uint8_t sizey)
{
    uint8_t t, temp, sizex;
    uint16_t num1;
    sizex = sizey / 2;
    num1 = num * 100;

    for (t = 0; t < len; t++) {
        temp = (num1 / mypow(10, len - t - 1)) % 10;
        if (t == (len - 2)) {
            LCD_ShowChar(x + (len - 2) * sizex, y, '.', fc, bc, sizey, 0);
            t++;
            len += 1;
        }
        LCD_ShowChar(x + t * sizex, y, temp + 48, fc, bc, sizey, 0);
    }
}

/* ----------------------- 图片显示 ----------------------- */
/* pic[]: RGB565(高字节在前), 长度 = length*width*2, 可位于 flash */
void LCD_ShowPicture(uint16_t x, uint16_t y, uint16_t length, uint16_t width,
                     const uint8_t pic[])
{
    LCD_Address_Set(x, y, x + length - 1, y + width - 1);
    LCD_Push(pic, (size_t)length * width * 2);
}
