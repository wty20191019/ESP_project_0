idf_component_register(
    SRC_DIRS "led" "wm8978"
    INCLUDE_DIRS "led" "wm8978"
    REQUIRES driver esp_driver_gpio led_strip esp_driver_i2c esp_driver_i2s esp_rom
)

component_compile_options(-ffast-math -O3 -Wno-error=format -Wno-format)

