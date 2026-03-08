#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_timer.h"
#include "esp_log.h"
#include "esp_err.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "math.h"

#include "my_mx1508.h"
#include "my_mpu6050.h"
#include "my_pid.h"

#define I2C_SDA_PIN    GPIO_NUM_21
#define I2C_SCL_PIN    GPIO_NUM_22
#define MOTOR_L_IN1    GPIO_NUM_32
#define MOTOR_L_IN2    GPIO_NUM_33
#define MOTOR_R_IN3    GPIO_NUM_25
#define MOTOR_R_IN4    GPIO_NUM_26

#define LOOP_DT        0.005f  
#define ANGLE_LIMIT    40.0f

static my_mpu6050_t mpu;
static mx1508_t motor;
static PIDController pid;

void mpu_task(void *arg)
{
    while (1)
    {
        my_mpu6050_read_raw(&mpu);
        my_mpu6050_convert_to_units(&mpu);
        my_mpu6050_update_complementary(&mpu, LOOP_DT, 0.98f);
        printf("Pitch: %.2f, Roll: %.2f\n", mpu.pitch, mpu.roll);

        vTaskDelay(pdMS_TO_TICKS(5));  
    }
}

void balance_task(void *arg)
{
    while (1)
    {
        my_mpu6050_read_raw(&mpu);
        my_mpu6050_convert_to_units(&mpu);
        my_mpu6050_update_complementary(&mpu, LOOP_DT, 0.98f);

        float roll = mpu.roll;

        if (fabs(roll - pid.setpoint) > ANGLE_LIMIT)
        {
            my_mx1508_set_speed(&motor, 0, 0);
            pid_reset(&pid);
        }
        else
        {
            float pwm = pid_compute(&pid, roll);
            static int log_count = 0;
            if (++log_count >= 80) {
                printf("Roll: %.2f PWM: %.2f\n", roll, pwm);
                log_count = 0;
            }

        
            my_mx1508_set_speed(&motor, -(int16_t)pwm, -(int16_t)pwm);
        }

        vTaskDelay(pdMS_TO_TICKS(5)); 
    }
}

void deadzone_test_task(void *arg)
{
    for(int i = 200; i <= 400; i += 5)
    {
        my_mx1508_set_speed(&motor, i, i);
        printf("Set PWM: %d\n", i);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = 0,
        .scl_io_num = I2C_SCL_PIN,
        .sda_io_num = I2C_SDA_PIN,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus_handle));

    ESP_ERROR_CHECK(my_mpu6050_init(bus_handle, &mpu));
    ESP_ERROR_CHECK(my_mpu6050_config(&mpu));

    vTaskDelay(pdMS_TO_TICKS(2000));
    my_mpu6050_calibrate(&mpu, 200);

    ESP_ERROR_CHECK(my_mx1508_init(&motor,
                                  MOTOR_L_IN1, MOTOR_L_IN2,
                                  MOTOR_R_IN3, MOTOR_R_IN4,
                                  1000));

    my_mx1508_set_deadzone(&motor, 270, 270);
    my_mx1508_set_slew(&motor, 60);
    my_mx1508_set_max_speed(&motor, 100);

    pid_init(&pid,
             6.0f,    // Kp 
             0.0f,    // Ki
             4.0f,    // Kd
             LOOP_DT,
             -1023,
             1023);

    pid.setpoint = -5.0f;

    pid_set_limits(&pid, 1023);

    xTaskCreatePinnedToCore(balance_task, "balance_task", 4096, NULL, 10, NULL, 1);
}