#include "lcd_init.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/spi_master.h"

/* ============================================================
 * ST7735 1.8 寸 TFT(128x160) 底层驱动
 * 参考: STM32F103 硬件 SPI + DMA 例程 HARDWARE/LCD/lcd_init.c
 *
 * 硬件方案:
 *   - SPI2_HOST + 内部 DMA (spi_master 驱动, polling 同步发送)
 *   - CS/DC/RST/BLK 均为软件控制 GPIO
 *   - 命令字节: DC=0 ; 数据字节: DC=1
 * ============================================================ */

#define TAG "LCD"

static spi_device_handle_t s_lcd_spi = NULL; /* SPI 设备句柄 */

/* 大块数据发送时的中转缓冲(必须在片内 RAM, DMA 才能访问) */
#define STAGE_SIZE 512
static uint8_t s_stage[STAGE_SIZE];

/* ----------------------- 底层 SPI 发送 ----------------------- */

/* 在 CS 已拉低的条件下, 发送一段数据(内部按 512B 分块, 全程保持 CS 低) */
static esp_err_t spi_tx_block(const uint8_t *data, size_t len)
{
    size_t done = 0;
    while (done < len) {
        size_t chunk = (len - done > STAGE_SIZE) ? STAGE_SIZE : (len - done);

        /* 数据可能位于 flash(如图片数组), 先拷贝到片内 RAM 再由 DMA 读取 */
        memcpy(s_stage, data + done, chunk);

        spi_transaction_t t = { 0 };
        t.length    = chunk * 8;      /* 长度以 bit 计 */
        t.tx_buffer = s_stage;

        esp_err_t ret = spi_device_polling_transmit(s_lcd_spi, &t);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "spi transmit failed: %s", esp_err_to_name(ret));
            return ret;
        }
        done += chunk;
    }
    return ESP_OK;
}

/* ----------------------- GPIO 片选辅助 ----------------------- */
static inline void cs_low(void)  { gpio_set_level(LCD_CS, 0); }
static inline void cs_high(void) { gpio_set_level(LCD_CS, 1); }
static inline void dc_cmd(void)  { gpio_set_level(LCD_DC, 0); }
static inline void dc_data(void) { gpio_set_level(LCD_DC, 1); }

/* ----------------------- GPIO 初始化 ----------------------- */
void LCD_GPIO_Init(void)
{
    /* 软件控制的 4 根线: CS/DC/RST/BLK */
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << LCD_CS) | (1ULL << LCD_DC) |
                        (1ULL << LCD_RST) | (1ULL << LCD_BLK),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io));

    /* 上电默认电平: CS 高(不选中)、DC 高、RST 高、背光关 */
    gpio_set_level(LCD_CS,  1);
    gpio_set_level(LCD_DC,  1);
    gpio_set_level(LCD_RST, 1);
    gpio_set_level(LCD_BLK, 0);
}

/* ----------------------- SPI 初始化 ----------------------- */
void LCD_SPI_Init(void)
{
    if (s_lcd_spi != NULL) {
        return; /* 已初始化 */
    }

    spi_bus_config_t bus = {
        .sclk_io_num     = LCD_SCK,
        .mosi_io_num     = LCD_MOSI,
        .miso_io_num     = -1,          /* 只写, 无 MISO */
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = STAGE_SIZE,
    };

    /* 与现有项目共用同一个 SPI 主机也无妨; LCD 独占 SPI2_HOST */
    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);

    spi_device_interface_config_t dev = {
        .clock_speed_hz = LCD_SPI_CLK_HZ,
        .mode           = 0,           /* CPOL=0 CPHA=0, ST7735 标准 */
        .spics_io_num   = -1,          /* CS 手动控制 */
        .queue_size     = 4,
        .flags          = SPI_DEVICE_HALFDUPLEX,
    };
    ret = spi_bus_add_device(SPI2_HOST, &dev, &s_lcd_spi);
    ESP_ERROR_CHECK(ret);
}

/* ----------------------- 字节级底层写 ----------------------- */
/* 写 1 字节命令 */
void LCD_WR_REG(uint8_t dat)
{
    dc_cmd();
    cs_low();
    spi_tx_block(&dat, 1);
    cs_high();
    dc_data();
}

/* 写 1 字节数据 */
void LCD_WR_DATA8(uint8_t dat)
{
    dc_data();
    cs_low();
    spi_tx_block(&dat, 1);
    cs_high();
}

/* 写 16 位数据(高字节在前) */
void LCD_WR_DATA(uint16_t dat)
{
    LCD_WR_DATA8(dat >> 8);
    LCD_WR_DATA8(dat & 0xFF);
}

/* 推送一大段像素/图片数据: 一次拉低 CS, 分块 DMA 发送, 直到全部完成 */
void LCD_Push(const uint8_t *buf, size_t len)
{
    dc_data();
    cs_low();
    spi_tx_block(buf, len);
    cs_high();
}

/* ----------------------- 设置显示窗口 ----------------------- */
void LCD_Address_Set(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    /* ST7735 非标准起始偏移与参考例程一致 */
#if (USE_HORIZONTAL == 0)
    LCD_WR_REG(0x2A); LCD_WR_DATA(x1 + 2); LCD_WR_DATA(x2 + 2);
    LCD_WR_REG(0x2B); LCD_WR_DATA(y1 + 1); LCD_WR_DATA(y2 + 1);
    LCD_WR_REG(0x2C);
#elif (USE_HORIZONTAL == 1)
    LCD_WR_REG(0x2A); LCD_WR_DATA(x1 + 2); LCD_WR_DATA(x2 + 2);
    LCD_WR_REG(0x2B); LCD_WR_DATA(y1 + 1); LCD_WR_DATA(y2 + 1);
    LCD_WR_REG(0x2C);
#elif (USE_HORIZONTAL == 2)
    LCD_WR_REG(0x2A); LCD_WR_DATA(x1 + 1); LCD_WR_DATA(x2 + 1);
    LCD_WR_REG(0x2B); LCD_WR_DATA(y1 + 2); LCD_WR_DATA(y2 + 2);
    LCD_WR_REG(0x2C);
#else
    LCD_WR_REG(0x2A); LCD_WR_DATA(x1 + 1); LCD_WR_DATA(x2 + 1);
    LCD_WR_REG(0x2B); LCD_WR_DATA(y1 + 2); LCD_WR_DATA(y2 + 2);
    LCD_WR_REG(0x2C);
#endif
}

/* ----------------------- 背光控制 ----------------------- */
void LCD_Backlight(uint8_t on)
{
    gpio_set_level(LCD_BLK, on ? 1 : 0);
}

/* ----------------------- 总初始化 ----------------------- */
void LCD_Init(void)
{
    LCD_GPIO_Init();
    LCD_SPI_Init();

    /* 复位脉冲 */
    gpio_set_level(LCD_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(LCD_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(100));

    /* 点亮背光 */
    LCD_Backlight(1);
    vTaskDelay(pdMS_TO_TICKS(100));

    /* ---------- ST7735S 初始化时序(与参考例程逐字节一致) ---------- */
    LCD_WR_REG(0x11);                       /* Sleep out */
    vTaskDelay(pdMS_TO_TICKS(120));

    /* Frame Rate */
    LCD_WR_REG(0xB1); LCD_WR_DATA8(0x05); LCD_WR_DATA8(0x3C); LCD_WR_DATA8(0x3C);
    LCD_WR_REG(0xB2); LCD_WR_DATA8(0x05); LCD_WR_DATA8(0x3C); LCD_WR_DATA8(0x3C);
    LCD_WR_REG(0xB3); LCD_WR_DATA8(0x05); LCD_WR_DATA8(0x3C); LCD_WR_DATA8(0x3C);
                     LCD_WR_DATA8(0x05); LCD_WR_DATA8(0x3C); LCD_WR_DATA8(0x3C);
    LCD_WR_REG(0xB4); LCD_WR_DATA8(0x03);   /* Dot inversion */

    /* Power Sequence */
    LCD_WR_REG(0xC0); LCD_WR_DATA8(0x28); LCD_WR_DATA8(0x08); LCD_WR_DATA8(0x04);
    LCD_WR_REG(0xC1); LCD_WR_DATA8(0xC0);
    LCD_WR_REG(0xC2); LCD_WR_DATA8(0x0D); LCD_WR_DATA8(0x00);
    LCD_WR_REG(0xC3); LCD_WR_DATA8(0x8D); LCD_WR_DATA8(0x2A);
    LCD_WR_REG(0xC4); LCD_WR_DATA8(0x8D); LCD_WR_DATA8(0xEE);
    LCD_WR_REG(0xC5); LCD_WR_DATA8(0x1A);   /* VCOM */

    /* MX, MY, RGB 模式 */
    LCD_WR_REG(0x36);
#if (USE_HORIZONTAL == 0)
    LCD_WR_DATA8(0x00);
#elif (USE_HORIZONTAL == 1)
    LCD_WR_DATA8(0xC0);
#elif (USE_HORIZONTAL == 2)
    LCD_WR_DATA8(0x70);
#else
    LCD_WR_DATA8(0xA0);
#endif

    /* Gamma Sequence */
    LCD_WR_REG(0xE0);
    LCD_WR_DATA8(0x04); LCD_WR_DATA8(0x22); LCD_WR_DATA8(0x07); LCD_WR_DATA8(0x0A);
    LCD_WR_DATA8(0x2E); LCD_WR_DATA8(0x30); LCD_WR_DATA8(0x25); LCD_WR_DATA8(0x2A);
    LCD_WR_DATA8(0x28); LCD_WR_DATA8(0x26); LCD_WR_DATA8(0x2E); LCD_WR_DATA8(0x3A);
    LCD_WR_DATA8(0x00); LCD_WR_DATA8(0x01); LCD_WR_DATA8(0x03); LCD_WR_DATA8(0x13);

    LCD_WR_REG(0xE1);
    LCD_WR_DATA8(0x04); LCD_WR_DATA8(0x16); LCD_WR_DATA8(0x06); LCD_WR_DATA8(0x0D);
    LCD_WR_DATA8(0x2D); LCD_WR_DATA8(0x26); LCD_WR_DATA8(0x23); LCD_WR_DATA8(0x27);
    LCD_WR_DATA8(0x27); LCD_WR_DATA8(0x25); LCD_WR_DATA8(0x2D); LCD_WR_DATA8(0x3B);
    LCD_WR_DATA8(0x00); LCD_WR_DATA8(0x01); LCD_WR_DATA8(0x04); LCD_WR_DATA8(0x13);

    LCD_WR_REG(0x3A); LCD_WR_DATA8(0x05);   /* 65K RGB565 */
    LCD_WR_REG(0x29);                       /* Display on */
}
