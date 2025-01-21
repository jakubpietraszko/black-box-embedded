idf_component_register(
    SRCS "final.c" "mpu6050.c"
    INCLUDE_DIRS "."
    PRIV_INCLUDE_DIRS "${IDF_PATH}/components/bt/host/bluedroid/api/include/api"
    REQUIRES nvs_flash driver bt mqtt esp_http_server esp_adc
)
