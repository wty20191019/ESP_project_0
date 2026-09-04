#ifndef __KEY_H__
#define __KEY_H__

#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ===== 按键引脚宏定义（带功能标识） ===== */
#define KEY_UP           GPIO_NUM_2      // 上
#define KEY_DOWN         GPIO_NUM_7      // 下
#define KEY_LEFT         GPIO_NUM_38     // 左
#define KEY_RIGHT        GPIO_NUM_39     // 右
#define KEY_MID          GPIO_NUM_40     // 中间确认
#define KEY_SET          GPIO_NUM_41     // 设置
#define KEY_RST          GPIO_NUM_47     // 复位

/* 按键总数 */
#define KEY_NUM_MAX     7

/* 按键编号枚举 */
typedef enum {
    KEY_ID_UP = 0,
    KEY_ID_DOWN,
    KEY_ID_LEFT,
    KEY_ID_RIGHT,
    KEY_ID_MID,
    KEY_ID_SET,
    KEY_ID_RST,
} key_id_t;

/* 按键事件类型 */
typedef enum {
    KEY_EVENT_PRESSED,      // 按下
    KEY_EVENT_RELEASED,     // 松开
    KEY_EVENT_CLICK,        // 单击
    KEY_EVENT_LONG_PRESS,   // 长按
} key_event_t;

/* 按键回调函数原型 */
typedef void (*key_callback_t)(key_id_t key_id, key_event_t event, void *user_data);

/**
 * @brief 初始化按键
 * @param callback  按键回调函数
 * @param user_data 用户自定义数据
 */
void key_init(key_callback_t callback, void *user_data);

/**
 * @brief 获取按键当前电平
 * @param key_id 按键编号
 * @return 0=按下，1=松开
 */
int key_get_level(key_id_t key_id);

#ifdef __cplusplus
}
#endif

#endif /* __KEY_H__ */




/*
#include "key.h"


static void my_key_callback(key_id_t key_id, key_event_t event, void *user_data)
{
    switch (event) {
        case KEY_EVENT_CLICK:
            ESP_LOGI("APP", "KEY%d 单击", key_id + 1);
            break;
        case KEY_EVENT_LONG_PRESS:
            ESP_LOGI("APP", "KEY%d 长按", key_id + 1);
            break;
        case KEY_EVENT_PRESSED:
            // 按下瞬间响应
            break;
        case KEY_EVENT_RELEASED:
            // 松开瞬间响应
            break;
    }
}

void app_main(void)
{
    key_init(my_key_callback, NULL);
    // ... 其他初始化 ...
}

*/
