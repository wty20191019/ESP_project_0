#include "wm8978_i2c.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_rom_sys.h"
#include "driver/i2c_master.h"

/* WM8978_I2C：ESP32-S3 I2C0 主机总线底层驱动，
 * 对应参考实验中的 IIC(myiic) + wm8978 写寄存器时序部分，
 * 此处改用 ESP-IDF 硬件 I2C 主机驱动实现。 */

static const char *TAG = "WM8978_I2C";

static i2c_master_bus_handle_t s_bus = NULL;    /* I2C0 主机总线句柄 */
static i2c_master_dev_handle_t s_dev = NULL;    /* WM8978 从机设备句柄 */

esp_err_t wm8978_i2c_init(void)
{
    if (s_dev != NULL) return ESP_OK;           /* 已初始化，直接返回 */

    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,                  /* I2C0 */
        .sda_io_num = WM_I2C_SDA,
        .scl_io_num = WM_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .trans_queue_depth = 0,                 /* 仅同步收发，无需后台队列 */
        .flags.enable_internal_pullup = true,   /* 使能内部上拉（板级最好再外加上拉） */
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_cfg, &s_bus),
                        TAG, "创建 I2C0 主机总线失败");

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = WM8978_I2C_ADDR,
        .scl_speed_hz = WM8978_I2C_FREQ_HZ,
    };
    esp_err_t err = i2c_master_bus_add_device(s_bus, &dev_cfg, &s_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "添加 WM8978 I2C 设备失败");
        i2c_del_master_bus(s_bus);
        s_bus = NULL;
        return err;
    }
    return ESP_OK;
}

esp_err_t wm8978_i2c_deinit(void)
{
    esp_err_t err = ESP_OK;
    if (s_dev != NULL) {
        err = i2c_master_bus_rm_device(s_dev);
        s_dev = NULL;
    }
    if (s_bus != NULL) {
        if (i2c_del_master_bus(s_bus) != ESP_OK) err = ESP_FAIL;
        s_bus = NULL;
    }
    return err;
}

esp_err_t wm8978_i2c_write_reg(uint8_t reg, uint16_t val)
{
    if (s_dev == NULL) return ESP_ERR_INVALID_STATE;

    /* WM8978 寄存器写协议：2 字节数据 */
    uint8_t buf[2];
    buf[0] = (uint8_t)((reg << 1) | ((val >> 8) & 0x01));   /* 寄存器地址 + 数值最高位 */
    buf[1] = (uint8_t)(val & 0xFF);                          /* 数值低 8 位 */

    /* 上电早期/总线毛刺会偶发 NACK：失败时复位总线并重试 */
    esp_err_t err = ESP_FAIL;
    for (int i = 0; i < WM8978_I2C_WRITE_RETRY; i++) {
        err = i2c_master_transmit(s_dev, buf, sizeof(buf), 100);
        if (err == ESP_OK) return ESP_OK;

        if (i + 1 < WM8978_I2C_WRITE_RETRY) {
            i2c_master_bus_reset(s_bus);        /* 释放可能被从机钳住的总线 */
            esp_rom_delay_us(2000);             /* 短暂等待从机就绪 */
        }
    }
    ESP_LOGE(TAG, "写寄存器 R%d = 0x%03X 失败(%d 次)", reg, val, WM8978_I2C_WRITE_RETRY);
    return err;
}
