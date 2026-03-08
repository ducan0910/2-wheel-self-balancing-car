idf_component_register(
    SRCS "my_mpu6050.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_timer
)