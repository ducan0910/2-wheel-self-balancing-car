idf_component_register(
    SRCS "my_pid.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_timer
)