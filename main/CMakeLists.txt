idf_component_register(
    SRCS "bsp.c" "batt.c" "main.c" "lsm6dso.c"
    INCLUDE_DIRS "."
    REQUIRES "nvs_flash" "bt" "driver" "esp_adc"
)