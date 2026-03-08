#pragma once

#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

typedef struct {
    gpio_num_t in1, in2, in3, in4;
    ledc_channel_t ch1, ch2, ch3, ch4;

    int16_t deadzone_l;
    int16_t deadzone_r;

    int16_t last_l;
    int16_t last_r;

    int16_t max_step;    
    uint8_t max_percent;
} mx1508_t;

esp_err_t my_mx1508_init(mx1508_t *dev,
                         gpio_num_t in1, gpio_num_t in2,
                         gpio_num_t in3, gpio_num_t in4,
                         uint32_t pwm_freq);

void my_mx1508_set_deadzone(mx1508_t *dev,
                            int16_t dz_l,
                            int16_t dz_r);

void my_mx1508_set_slew(mx1508_t *dev, int16_t max_step);
void my_mx1508_set_max_speed(mx1508_t *dev, uint8_t percent);

esp_err_t my_mx1508_set_speed(mx1508_t *dev,
                             int16_t left,
                             int16_t right);

esp_err_t my_mx1508_coast(mx1508_t *dev);