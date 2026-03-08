#include "my_mpu6050.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define MPU_ADDR                  0x68     // Địa chỉ I2C của MPU6050

#define MPU6050_REG_PWR_MGMT_1    0x6B     //107 
#define MPU6050_REG_WHO_AM_I      0x75     //117
#define MPU6050_REG_ACCEL_XOUT_H  0x3B     //59 .Từ đấy đọc liên tục 14 byte đến GYRO_ZOUT_L
#define MPU6050_REG_GYRO_CONFIG   0x1B     //27
#define MPU6050_REG_ACCEL_CONFIG  0x1C     //28
#define MPU6050_REG_SAMPLRT_DIV   0x19     //25
#define MPU6050_REG_CONFIG        0x1A     //26

#define RAD_TO_DEG                57.2957795f


esp_err_t my_mpu6050_init(i2c_master_bus_handle_t bus_handle, my_mpu6050_t *mpu)
{
    memset(mpu, 0, sizeof(my_mpu6050_t));

    // Thêm thiết bị MPU6050 vào bus I2C
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_7,
        .device_address = MPU_ADDR, 
        .scl_speed_hz = 400000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_config, &mpu->dev));
    ESP_LOGI("MPU6050", "I2C of MPU6050 initialized successfully");
    return ESP_OK;
}

static esp_err_t write_reg(uint8_t reg, uint8_t value, my_mpu6050_t *mpu)
{
    uint8_t data[2] = {reg, value};
    return i2c_master_transmit(mpu->dev, data, 2, pdMS_TO_TICKS(100));
}

esp_err_t my_mpu6050_config(my_mpu6050_t *mpu)
{
    ESP_ERROR_CHECK(write_reg(MPU6050_REG_PWR_MGMT_1, 0x80, mpu));      // Reset device
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_ERROR_CHECK(write_reg(MPU6050_REG_PWR_MGMT_1, 0x01, mpu));      // SLEEP = 0, CLKSEL = 1 (chọn X axis gyroscope làm nguồn xung nhịp)
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_ERROR_CHECK(write_reg(MPU6050_REG_ACCEL_CONFIG, 0x00, mpu));    // ±2g
    ESP_ERROR_CHECK(write_reg(MPU6050_REG_GYRO_CONFIG, 0x00, mpu));     // ±250°/s
    ESP_ERROR_CHECK(write_reg(MPU6050_REG_CONFIG, 0x03, mpu));          // DLPF_CFG = 3
    ESP_ERROR_CHECK(write_reg(MPU6050_REG_SAMPLRT_DIV, 0x04, mpu));     // SMPLRT_DIV = 4 → 1kHz / (1+4) = 200 Hz 

    //Kiểm tra kết nối bằng cách đọc WHO_AM_I
    uint8_t who;
    uint8_t cmd = MPU6050_REG_WHO_AM_I;
    ESP_ERROR_CHECK(i2c_master_transmit_receive(mpu->dev, &cmd, 1, &who, 1, 1000));
    ESP_LOGI("MPU6050", "WHO_AM_I = 0x%02X", who);
    ESP_LOGI("MPU6050", "MPU6050 đã cấu hình thành công");

    return ESP_OK;
} 

esp_err_t my_mpu6050_read_raw(my_mpu6050_t *mpu)
{
    uint8_t buffer[14];
    uint8_t addr = MPU6050_REG_ACCEL_XOUT_H;     
    ESP_ERROR_CHECK(i2c_master_transmit_receive(mpu->dev, &addr, 1, buffer, 14, 1000));

    mpu->accel_raw[0] = (int16_t)(buffer[0] << 8) | buffer[1];
    mpu->accel_raw[1] = (int16_t)(buffer[2] << 8) | buffer[3];
    mpu->accel_raw[2] = (int16_t)(buffer[4] << 8) | buffer[5];
    mpu->temp_raw = (int16_t)(buffer[6] << 8) | buffer[7];
    mpu->gyro_raw[0] = (int16_t)(buffer[8] << 8) | buffer[9];
    mpu->gyro_raw[1] = (int16_t)(buffer[10] << 8) | buffer[11];
    mpu->gyro_raw[2] = (int16_t)(buffer[12] << 8) | buffer[13];

    return ESP_OK;
}

void my_mpu6050_convert_to_units(my_mpu6050_t *mpu)
{
    mpu->accel_g[0] = mpu->accel_raw[0] / 16384.0f;
    mpu->accel_g[1] = mpu->accel_raw[1] / 16384.0f;
    mpu->accel_g[2] = mpu->accel_raw[2] / 16384.0f;

    mpu->gyro_dps[0] = mpu->gyro_raw[0] / 131.0f;
    mpu->gyro_dps[1] = mpu->gyro_raw[1] / 131.0f;
    mpu->gyro_dps[2] = mpu->gyro_raw[2] / 131.0f;

    mpu->temp_c = mpu->temp_raw / 340.0f + 36.53f;
}

void my_mpu6050_update_complementary(my_mpu6050_t *mpu, float dt, float alpha)
{
    float accel_roll = atan2f(mpu->accel_g[1], 
                             sqrtf(mpu->accel_g[0]*mpu->accel_g[0] + 
                                   mpu->accel_g[2]*mpu->accel_g[2])) * RAD_TO_DEG;
    float accel_pitch = atan2f(-mpu->accel_g[0], 
                             sqrtf(mpu->accel_g[1]*mpu->accel_g[1] + 
                                   mpu->accel_g[2]*mpu->accel_g[2])) * RAD_TO_DEG;
    
    mpu->roll = alpha * (mpu->roll + mpu->gyro_dps[0] * dt) + (1.0f - alpha) * accel_roll;
    mpu->pitch = alpha * (mpu->pitch + mpu->gyro_dps[1] * dt) + (1.0f - alpha) * accel_pitch;
}

void my_mpu6050_update_kalman(my_mpu6050_t *mpu);
esp_err_t my_mpu6050_calibrate(my_mpu6050_t *mpu, uint32_t samples)
{
    float sum_accel[3] = {0}, sum_gyro[3] = {0};
    for(int i = 0; i < samples; i++)
    {
        ESP_ERROR_CHECK(my_mpu6050_read_raw(mpu));
        for(int j = 0; j < 3; j++)
        {
            sum_accel[j] += mpu->accel_raw[j];
            sum_gyro[j] += mpu->gyro_raw[j];
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    for(int j = 0; j < 3; j++)
    {
        mpu->accel_offset[j] = sum_accel[j] / samples;
        mpu->gyro_offset[j] = sum_gyro[j] / samples;
    }
    mpu->accel_offset[2] -= 16384; 
    return ESP_OK;
}
esp_err_t my_mpu6050_set_IT(my_mpu6050_t *mpu, gpio_num_t int_pin);