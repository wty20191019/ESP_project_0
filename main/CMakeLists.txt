idf_component_register(
    SRCS "main.c" "ft8_app.c" "qso_log.c"
    INCLUDE_DIRS "."
    REQUIRES BSP ft8_lib esp_timer fatfs vfs wear_levelling esp_partition
)
