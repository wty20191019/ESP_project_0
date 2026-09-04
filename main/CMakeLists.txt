idf_component_register(
    SRCS "main.c" "ft8_app.c"
    INCLUDE_DIRS "."
    REQUIRES BSP ft8_lib esp_timer
)
