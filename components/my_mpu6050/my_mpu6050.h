#pragma once

#include "esp_err.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"


typedef struct my_mpu6050
{
    i2c_master_dev_handle_t dev;
    float pitch, roll;

    float accel_raw[3];
    float gyro_raw[3];
    float temp_raw;

    float accel_g[3];
    float gyro_dps[3];
    float temp_c;

    float accel_offset[3];
    float gyro_offset[3];

    uint64_t last_update_us;
    gpio_num_t int_pin;
} my_mpu6050_t;

/**
 * @brief Khởi tạo MPU6050
 * 
 * @param bus_handle 
 * @param mpu 
 * @return esp_err_t 
 */
esp_err_t my_mpu6050_init(i2c_master_bus_handle_t bus_handle, my_mpu6050_t *mpu);

/**
 * @brief Cấu hình MPU6050
 * 
 * @param mpu 
 * @return esp_err_t 
 */
esp_err_t my_mpu6050_config(my_mpu6050_t *mpu);

/**
 * @brief Đọc dữ liệu thô từ MPU6050 như x, y, z accelerometer và gyroscope
 * 
 * @param mpu 
 * @return esp_err_t 
 */
esp_err_t my_mpu6050_read_raw(my_mpu6050_t *mpu);

/**
 * @brief Chuyển đổi dữ liệu thô sang pitch và roll
 * 
 * @param mpu 
 */
void my_mpu6050_convert_to_units(my_mpu6050_t *mpu);

/**
 * @brief Làm mượt dữ liệu pitch và roll bằng bộ lọc bổ sung
 * 
 * @param mpu 
 * @param dt thời gian lấy mẫu
 * @param alpha hệ số 
 */
void my_mpu6050_update_complementary(my_mpu6050_t *mpu, float dt, float alpha);

/**
 * @brief Làm mượt dữ liệu pitch và roll bằng bộ lọc Kalman
 * 
 * @param mpu 
 */
void my_mpu6050_update_kalman(my_mpu6050_t *mpu);

/**
 * @brief Điều chỉnh MPU6050 để loại bỏ sai số bằng cách lấy mẫu nhiều lần
 * 
 * @param mpu 
 * @param samples 
 * @return esp_err_t 
 */
esp_err_t my_mpu6050_calibrate(my_mpu6050_t *mpu, uint32_t samples);

/**
 * @brief NGăt cấu hình ngắt từ MPU6050 đến chân GPIO được chỉ định
 * 
 * @param mpu 
 * @param int_pin 
 * @return esp_err_t 
 */
esp_err_t my_mpu6050_set_IT(my_mpu6050_t *mpu, gpio_num_t int_pin);
