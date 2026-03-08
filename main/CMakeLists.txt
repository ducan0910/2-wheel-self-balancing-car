idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES my_mpu6050 my_mx1508 my_pid)