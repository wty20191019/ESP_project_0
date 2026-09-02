idf_component_register(
    SRC_DIRS "led"
    INCLUDE_DIRS "led"
    REQUIRES driver esp_driver_gpio led_strip
)

component_compile_options(-ffast-math -O3 -Wno-error=format -Wno-format)

