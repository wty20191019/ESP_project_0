#include "lcd.h"
#include "lcdfont.h"

#include <stdlib.h>
#include "esp_log.h"

/* ============================================================
 * ST7735 1.8 寸 TFT(128x160) 上层绘图/字符 API —— 离屏整帧缓冲版
 *
 * 方案 B:
 *   所有绘图函数(点/线/面/字符/图片)先写入一片整帧 RAM 缓冲
 *   (逻辑坐标 row-major, RGB565 高字节在前), 全程不触碰屏幕;
 *   每帧画完调用 LCD_Flush(), 整帧一次窗口设置 + 连续 DMA 推送,
 *   从而消除逐行涂写造成的"扫描/闪烁"观感。
 * ============================================================ */

#define TAG "LCD"

/* 帧缓冲: LCD_W x LCD_H x 2B (128x160 => 40KB) */
static uint8_t *s_fb = NULL;
static bool s_fb_nomem_logged = false;

/* 连续写窗口状态(模拟屏幕自动换行的写入顺序, 供字符/图片使用) */
static uint16_t s_wx0, s_wy0, s_wx1, s_wy1;
static uint32_t s_wpos;

/* ------------------- 帧缓冲基础操作 ------------------- */

/* 首次使用绘图函数时分配缓冲(LCD_Push 内部会再拷贝到片内小缓冲, 内存位置不限) */
static bool fb_ensure(void)
{
    if (s_fb) {
        return true;
    }
    size_t sz = (size_t)LCD_W * LCD_H * 2;
    s_fb = malloc(sz);
    if (!s_fb) {
        if (!s_fb_nomem_logged) {
            ESP_LOGE(TAG, "no memory for %u-byte frame buffer", (unsigned)sz);
            s_fb_nomem_logged = true;
        }
        return false;
    }
    return true;
}

/* 写单个像素(越界丢弃) */
static void fb_pixel(int x, int y, uint16_t color)
{
    if (!s_fb) return;
    if (x < 0 || y < 0 || x >= LCD_W || y >= LCD_H) return;
    size_t o = ((size_t)y * LCD_W + (size_t)x) * 2;
    s_fb[o]     = color >> 8;
    s_fb[o + 1] = color & 0xFF;
}

/* 开启"连续写"窗口: 像素将按 row-major 顺序填入窗口 */
static void fb_win_begin(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    s_wx0 = x1;
    s_wy0 = y1;
    s_wx1 = x2;
    s_wy1 = y2;
    s_wpos = 0;
}

/* 在连续窗口内追加一个像素(满一窗后丢弃越界部分) */
static void fb_win_pixel(uint16_t color)
{
    if (!s_fb) return;
    uint32_t w = (uint32_t)s_wx1 - s_wx0 + 1;
    uint32_t h = (uint32_t)s_wy1 - s_wy0 + 1;
    if (s_wpos >= w * h) return;

    int x = s_wx0 + (int)(s_wpos % w);
    int y = s_wy0 + (int)(s_wpos / w);
    s_wpos++;
    fb_pixel(x, y, color);
}

/* ============================================================
 * 公开 API
 * ============================================================ */

/* 把整帧缓冲一次性推送显示 */
void LCD_Flush(void)
{
    if (!fb_ensure()) return;
    LCD_Address_Set(0, 0, LCD_W - 1, LCD_H - 1);   /* 全屏窗口 + RAMWR */
    LCD_Push(s_fb, (size_t)LCD_W * LCD_H * 2);     /* 连续 DMA 推整帧 */
}

/* 清空缓冲(不直接上屏) */
void LCD_Clear(uint16_t color)
{
    if (!fb_ensure()) return;
    for (int y = 0; y < LCD_H; y++) {
        for (int x = 0; x < LCD_W; x++) {
            fb_pixel(x, y, color);
        }
    }
}

/* ----------------------- 基础图形 ----------------------- */

/* 区域填充: [xsta,xend) x [ysta,yend), 不包含右/下边界 */
void LCD_Fill(uint16_t xsta, uint16_t ysta, uint16_t xend, uint16_t yend, uint16_t color)
{
    if (!fb_ensure()) return;
    for (uint16_t y = ysta; y < yend; y++) {
        for (uint16_t x = xsta; x < xend; x++) {
            fb_pixel(x, y, color);
        }
    }
}

/* 画一个点 */
void LCD_DrawPoint(uint16_t x, uint16_t y, uint16_t color)
{
    if (!fb_ensure()) return;
    fb_pixel(x, y, color);
}

/* 画线 (Bresenham, 水平/垂直线走整块填充) */
void LCD_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    uint16_t t;
    int xerr = 0, yerr = 0, delta_x, delta_y, distance;
    int incx, incy, uRow, uCol;

    if (y1 == y2) {
        if (x1 > x2) { t = x1; x1 = x2; x2 = t; }
        LCD_Fill(x1, y1, x2 + 1, y1 + 1, color);
        return;
    }
    if (x1 == x2) {
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
        if ((a * a + b * b) > (r * r)) {
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
    uint16_t i, typeface_num;
    uint16_t x0 = x;

    if (!fb_ensure()) return;

    sizex = sizey / 2;
    typeface_num = (sizex / 8 + ((sizex % 8) ? 1 : 0)) * sizey;
    num = num - ' ';

    if (mode == 0) {
        /* 实心模式: 整窗连续写(顺序与直接推屏一致) */
        fb_win_begin(x, y, x + sizex - 1, y + sizey - 1);

        for (i = 0; i < typeface_num; i++) {
            if (sizey == 12)       temp = ascii_1206[num][i];
            else if (sizey == 16)  temp = ascii_1608[num][i];
            else if (sizey == 24)  temp = ascii_2412[num][i];
            else if (sizey == 32)  temp = ascii_3216[num][i];
            else return;

            for (t = 0; t < 8; t++) {
                if (temp & (0x01 << t)) fb_win_pixel(fc);
                else                    fb_win_pixel(bc);
                m++;
                if (m % sizex == 0) {
                    m = 0;
                    break;
                }
            }
        }
    } else {
        /* 透明叠加模式: 只写字形点 */
        for (i = 0; i < typeface_num; i++) {
            if (sizey == 12)       temp = ascii_1206[num][i];
            else if (sizey == 16)  temp = ascii_1608[num][i];
            else if (sizey == 24)  temp = ascii_2412[num][i];
            else if (sizey == 32)  temp = ascii_3216[num][i];
            else return;

            for (t = 0; t < 8; t++) {
                if (temp & (0x01 << t)) fb_pixel(x, y, fc);
                x++;
                if ((x - x0) == sizex) {
                    x = x0;
                    y++;
                    break;
                }
            }
        }
    }
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

/* 显示整数, len 为总位数(前导空格补齐) */
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
    if (!fb_ensure()) return;
    fb_win_begin(x, y, x + length - 1, y + width - 1);

    size_t bytes = (size_t)length * width * 2;
    for (size_t p = 0; p + 1 < bytes; p += 2) {
        uint16_t c = ((uint16_t)pic[p] << 8) | pic[p + 1];
        fb_win_pixel(c);
    }
}
