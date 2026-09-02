#include "led.h"
#include "esp_log.h"

static const char *TAG = "LED";
static led_strip_handle_t strip = NULL;

void led_init(void)
{
    if (strip != NULL) return;

    led_strip_config_t strip_cfg = {
        .strip_gpio_num = LED_GPIO_PIN,                                 // 数据线连接的 GPIO 引脚号
        .max_leds = LED_STRIP_LEN,                                      // 灯带上串联的 LED 数量
        .led_model = LED_MODEL_WS2812,                                  // LED 芯片型号（SK68xx 兼容 WS2812）
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,    // 颜色分量顺序：绿-红-蓝
        .flags.invert_out = false,                                      // 输出信号是否反转（false = 不反转）        
    };

    led_strip_rmt_config_t rmt_cfg = {
        .clk_src = RMT_CLK_SRC_DEFAULT,                                 // 使用默认 RMT 时钟源
        .resolution_hz = RMT_RES_HZ,                                    // RMT 分辨率
        .flags.with_dma = false,                                        // esp32 必须 false；esp32s3 单颗也 false//// 不使用 DMA（单颗 LED 无需 DMA）
    };

    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_cfg, &rmt_cfg, &strip));    // 创建 RMT 设备，将配置应用到硬件
    led_clear();                                                                // 初始化完成后，先将所有 LED 熄灭
}

void led_set_rgb(uint8_t r, uint8_t g, uint8_t b)
{
    if (!strip) return;
    ESP_ERROR_CHECK(led_strip_set_pixel(strip, 0, r, g, b));
    ESP_ERROR_CHECK(led_strip_refresh(strip));
}

void led_clear(void)
{
    if (!strip) return;
    ESP_ERROR_CHECK(led_strip_clear(strip));
}

