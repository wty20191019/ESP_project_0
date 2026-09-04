#include "key.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "key";

/* ===== 按键 GPIO 映射表（由宏定义构建） ===== */
static const gpio_num_t s_key_gpios[KEY_NUM_MAX] = {
    KEY_UP,         // KEY_ID_UP
    KEY_DOWN,       // KEY_ID_DOWN
    KEY_LEFT,       // KEY_ID_LEFT
    KEY_RIGHT,      // KEY_ID_RIGHT
    KEY_MID,        // KEY_ID_MID
    KEY_SET,        // KEY_ID_SET
    KEY_RST,        // KEY_ID_RST
};

/* 按键状态结构体 */
typedef struct {
    uint8_t last_level;          // 上一次采样的电平
    uint8_t stable_level;        // 防抖后的稳定电平
    uint8_t debounce_cnt;        // 防抖计数
    uint32_t press_start_ms;     // 按下时刻（用于长按检测）
    uint8_t long_press_reported; // 长按事件是否已上报
} key_state_t;

static key_state_t s_key_states[KEY_NUM_MAX];
static key_callback_t s_callback = NULL;
static void *s_user_data = NULL;

/* 防抖阈值（连续采样次数，每次约 10ms） */
#define DEBOUNCE_THRESHOLD  3
/* 长按判定时间（毫秒） */
#define LONG_PRESS_MS       1000

/* 按键扫描任务 */
static void key_scan_task(void *arg)
{
    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));

        for (int id = 0; id < KEY_NUM_MAX; id++) {
            int level = gpio_get_level(s_key_gpios[id]);

            /* ---- 防抖 ---- */
            if (level == s_key_states[id].last_level) {
                if (s_key_states[id].debounce_cnt < DEBOUNCE_THRESHOLD) {
                    s_key_states[id].debounce_cnt++;
                }
                if (s_key_states[id].debounce_cnt >= DEBOUNCE_THRESHOLD) {
                    uint8_t prev_stable = s_key_states[id].stable_level;
                    s_key_states[id].stable_level = level;

                    if (prev_stable != level) {
                        if (level == 0) {
                            /* 按下（低电平） */
                            s_key_states[id].press_start_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                            s_key_states[id].long_press_reported = 0;
                            if (s_callback) {
                                s_callback((key_id_t)id, KEY_EVENT_PRESSED, s_user_data);
                            }
                        } else {
                            /* 松开（高电平） */
                            uint32_t hold_ms = (xTaskGetTickCount() * portTICK_PERIOD_MS) - s_key_states[id].press_start_ms;
                            if (hold_ms < LONG_PRESS_MS) {
                                if (s_callback) {
                                    s_callback((key_id_t)id, KEY_EVENT_CLICK, s_user_data);
                                }
                            }
                            if (s_callback) {
                                s_callback((key_id_t)id, KEY_EVENT_RELEASED, s_user_data);
                            }
                        }
                    }
                }
            } else {
                s_key_states[id].debounce_cnt = 0;
            }

            /* ---- 长按检测 ---- */
            if (s_key_states[id].stable_level == 0 && !s_key_states[id].long_press_reported) {
                uint32_t hold_ms = (xTaskGetTickCount() * portTICK_PERIOD_MS) - s_key_states[id].press_start_ms;
                if (hold_ms >= LONG_PRESS_MS) {
                    s_key_states[id].long_press_reported = 1;
                    if (s_callback) {
                        s_callback((key_id_t)id, KEY_EVENT_LONG_PRESS, s_user_data);
                    }
                }
            }

            s_key_states[id].last_level = level;
        }
    }
}

/* 初始化按键 */
void key_init(key_callback_t callback, void *user_data)
{
    s_callback = callback;
    s_user_data = user_data;

    /* 配置所有按键 GPIO 为输入 + 内部上拉 */
    gpio_config_t io_conf = {
        .intr_type    = GPIO_INTR_DISABLE,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };

    for (int i = 0; i < KEY_NUM_MAX; i++) {
        io_conf.pin_bit_mask = (1ULL << s_key_gpios[i]);
        gpio_config(&io_conf);
    }

    /* 清空状态 */
    memset(s_key_states, 0, sizeof(s_key_states));

    /* 读取初始电平 */
    for (int i = 0; i < KEY_NUM_MAX; i++) {
        s_key_states[i].last_level   = gpio_get_level(s_key_gpios[i]);
        s_key_states[i].stable_level = s_key_states[i].last_level;
    }

    /* 创建按键扫描任务 */
    xTaskCreatePinnedToCore(key_scan_task, "key_scan", 4096, NULL, 1, NULL, 1);

    ESP_LOGI(TAG, "按键初始化完成，共 %d 个按键", KEY_NUM_MAX);
}

/* 获取按键当前稳定电平 */
int key_get_level(key_id_t key_id)
{
    if (key_id >= KEY_NUM_MAX) return 1;
    return s_key_states[key_id].stable_level;
}