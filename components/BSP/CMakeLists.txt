idf_component_register(
    SRC_DIRS        "led" "wm8978" "key" "gps" "LCD_st7735"
    INCLUDE_DIRS    "led" "wm8978" "key" "gps" "LCD_st7735"
    REQUIRES driver esp_driver_gpio led_strip esp_driver_i2c esp_driver_i2s esp_rom   esp_driver_uart  esp_timer freertos  log  esp_driver_spi    
)

component_compile_options(-ffast-math -O3 -Wno-error=format -Wno-format)

