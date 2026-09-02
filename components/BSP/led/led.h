#pragma once
#include "driver/gpio.h"
#include "led_strip.h"

#define LED_GPIO_PIN    GPIO_NUM_48
#define LED_STRIP_LEN   1
#define RMT_RES_HZ      (10 * 1000 * 1000)

void led_init(void);
void led_set_rgb(uint8_t r, uint8_t g, uint8_t b);
void led_clear(void);

