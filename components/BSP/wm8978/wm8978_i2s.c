#include "wm8978_i2s.h"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/i2s_common.h"

/* WM8978_I2S：ESP32-S3 I2S0 标准模式主机底层驱动，
 * 对应参考实验中的 I2S2/I2S2ext + DMA 部分。
 * ESP-IDF 采用 DMA 环形缓冲，通过 i2s_channel_write/read 阻塞收发。
 * WM8978 作为 I2S 从机，由本驱动(Master)输出 MCLK/BCLK/LRCK。 */

static const char *TAG = "WM8978_I2S";

static i2s_chan_handle_t s_tx = NULL;   /* TX(播放) 通道句柄 */
static i2s_chan_handle_t s_rx = NULL;   /* RX(录音) 通道句柄 */
static bool s_inited = false;           /* 是否已初始化 */
static bool s_tx_en = false;            /* TX 通道运行标志 */
static bool s_rx_en = false;            /* RX 通道运行标志 */

esp_err_t wm8978_i2s_init(uint32_t sample_rate_hz)
{
    if (s_inited) return ESP_OK;        /* 已初始化，幂等返回 */

    /* 分配 I2S0 主机 TX/RX 通道，构成全双工 */
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = WM8978_I2S_DMA_DESC_NUM;
    chan_cfg.dma_frame_num = WM8978_I2S_DMA_FRAME_NUM;
    ESP_RETURN_ON_ERROR(i2s_new_channel(&chan_cfg, &s_tx, &s_rx),
                        TAG, "创建 I2S0 通道失败");

    /* 标准模式：飞利浦(I2S)标准，16 位双声道，WM8978 用作从机 */
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate_hz),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(WM8978_I2S_DATA_BITS, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = WM_I2S_MCLK,
            .bclk = WM_I2S_BCLK,
            .ws   = WM_I2S_LRCK,
            .dout = WM_I2S_DOUT,
            .din  = WM_I2S_DIN,
        },
    };

    esp_err_t err = i2s_channel_init_std_mode(s_tx, &std_cfg);
    if (err == ESP_OK) err = i2s_channel_init_std_mode(s_rx, &std_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2S0 标准模式初始化失败");
        i2s_del_channel(s_tx);
        i2s_del_channel(s_rx);
        s_tx = s_rx = NULL;
        return err;
    }
    s_inited = true;
    ESP_LOGI(TAG, "I2S0 初始化完成，采样率 %lu Hz", (unsigned long)sample_rate_hz);
    return ESP_OK;
}

esp_err_t wm8978_i2s_deinit(void)
{
    esp_err_t err = ESP_OK;
    if (!s_inited) return ESP_OK;

    if (s_tx_en) err |= i2s_channel_disable(s_tx);
    if (s_rx_en) err |= i2s_channel_disable(s_rx);
    err |= i2s_del_channel(s_tx);
    err |= i2s_del_channel(s_rx);
    s_tx = s_rx = NULL;
    s_tx_en = s_rx_en = false;
    s_inited = false;
    return (err == ESP_OK) ? ESP_OK : ESP_FAIL;
}

esp_err_t wm8978_i2s_set_sample_rate(uint32_t sample_rate_hz)
{
    if (!s_inited) return ESP_ERR_INVALID_STATE;

    /* 记录并停止正在运行的通道 */
    bool re_tx = s_tx_en;
    bool re_rx = s_rx_en;
    if (re_tx) i2s_channel_disable(s_tx);
    if (re_rx) i2s_channel_disable(s_rx);
    s_tx_en = s_rx_en = false;

    /* 重新配置时钟（两通道共用 BCLK/WS，采样率需一致） */
    i2s_std_clk_config_t clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate_hz);
    esp_err_t err = i2s_channel_reconfig_std_clock(s_tx, &clk_cfg);
    if (err == ESP_OK) err = i2s_channel_reconfig_std_clock(s_rx, &clk_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "重设采样率 %lu Hz 失败", (unsigned long)sample_rate_hz);
        return err;
    }

    /* 恢复原运行状态 */
    if (re_tx) err |= i2s_channel_enable(s_tx);
    if (re_rx) err |= i2s_channel_enable(s_rx);
    s_tx_en = re_tx;
    s_rx_en = re_rx;
    return (err == ESP_OK) ? ESP_OK : ESP_FAIL;
}

esp_err_t wm8978_i2s_start_tx(void)
{
    if (!s_inited) return ESP_ERR_INVALID_STATE;
    ESP_RETURN_ON_ERROR(i2s_channel_enable(s_tx), TAG, "启动 TX 通道失败");
    s_tx_en = true;
    return ESP_OK;
}

esp_err_t wm8978_i2s_stop_tx(void)
{
    if (!s_inited) return ESP_ERR_INVALID_STATE;
    ESP_RETURN_ON_ERROR(i2s_channel_disable(s_tx), TAG, "停止 TX 通道失败");
    s_tx_en = false;
    return ESP_OK;
}

esp_err_t wm8978_i2s_start_rx(void)
{
    if (!s_inited) return ESP_ERR_INVALID_STATE;
    ESP_RETURN_ON_ERROR(i2s_channel_enable(s_rx), TAG, "启动 RX 通道失败");
    s_rx_en = true;
    return ESP_OK;
}

esp_err_t wm8978_i2s_stop_rx(void)
{
    if (!s_inited) return ESP_ERR_INVALID_STATE;
    ESP_RETURN_ON_ERROR(i2s_channel_disable(s_rx), TAG, "停止 RX 通道失败");
    s_rx_en = false;
    return ESP_OK;
}

esp_err_t wm8978_i2s_write(const void *src, size_t size, size_t *bytes_written, uint32_t timeout_ms)
{
    if (!s_inited || src == NULL) return ESP_ERR_INVALID_STATE;
    return i2s_channel_write(s_tx, src, size, bytes_written, timeout_ms);
}

esp_err_t wm8978_i2s_read(void *dest, size_t size, size_t *bytes_read, uint32_t timeout_ms)
{
    if (!s_inited || dest == NULL) return ESP_ERR_INVALID_STATE;
    return i2s_channel_read(s_rx, dest, size, bytes_read, timeout_ms);
}

i2s_chan_handle_t wm8978_i2s_get_tx_handle(void)
{
    return s_tx;
}

i2s_chan_handle_t wm8978_i2s_get_rx_handle(void)
{
    return s_rx;
}
