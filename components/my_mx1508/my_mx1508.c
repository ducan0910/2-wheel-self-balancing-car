#include "my_mx1508.h"
#include <stdlib.h>
#include <stdint.h>
#include "driver/ledc.h"
#include "driver/gpio.h"

#define PWM_MAX 1023

static inline int16_t clamp(int16_t v, int16_t min, int16_t max)
{
    if (v > max) return max;
    if (v < min) return min;
    return v;
}

static inline int16_t apply_deadzone(int16_t v, int16_t dz)
{
    if (v == 0) return 0;
    return (v > 0) ? (v + dz) : (v - dz);
}

static inline int16_t slew(int16_t target, int16_t prev, int16_t step)
{
    if (target > prev + step) return prev + step;
    if (target < prev - step) return prev - step;
    return target;
}

static void drive(ledc_channel_t chA,
                  ledc_channel_t chB,
                  int16_t pwm)
{
    uint32_t duty = (pwm >= 0) ? pwm : -pwm;

    if (pwm > 0) {
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, chA, duty);
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, chB, 0);
    } else if (pwm < 0) {
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, chA, 0);
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, chB, duty);
    } else {
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, chA, 0);
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, chB, 0);
    }

    ledc_update_duty(LEDC_HIGH_SPEED_MODE, chA);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, chB);
}

esp_err_t my_mx1508_init(mx1508_t *dev,
                         gpio_num_t in1, gpio_num_t in2,
                         gpio_num_t in3, gpio_num_t in4,
                         uint32_t pwm_freq)
{
    dev->in1 = in1; dev->in2 = in2;
    dev->in3 = in3; dev->in4 = in4;

    dev->ch1 = LEDC_CHANNEL_0;
    dev->ch2 = LEDC_CHANNEL_1;
    dev->ch3 = LEDC_CHANNEL_2;
    dev->ch4 = LEDC_CHANNEL_3;

    dev->deadzone_l = 0;
    dev->deadzone_r = 0;
    dev->max_step   = 40;
    dev->max_percent = 100;

    dev->last_l = 0;
    dev->last_r = 0;

    /* Timer */
    ledc_timer_config_t t = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = pwm_freq,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&t);

    ledc_channel_config_t c = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE,
        .duty = 0,
        .hpoint = 0
    };

    c.channel = dev->ch1; c.gpio_num = in1; ledc_channel_config(&c);
    c.channel = dev->ch2; c.gpio_num = in2; ledc_channel_config(&c);
    c.channel = dev->ch3; c.gpio_num = in3; ledc_channel_config(&c);
    c.channel = dev->ch4; c.gpio_num = in4; ledc_channel_config(&c);

    return my_mx1508_coast(dev);
}

void my_mx1508_set_deadzone(mx1508_t *dev, int16_t dz_l, int16_t dz_r)
{
    dev->deadzone_l = dz_l;
    dev->deadzone_r = dz_r;
}

void my_mx1508_set_slew(mx1508_t *dev, int16_t max_step)
{
    dev->max_step = max_step;
}

void my_mx1508_set_max_speed(mx1508_t *dev, uint8_t percent)
{
    dev->max_percent = (percent > 100) ? 100 : percent;
}

esp_err_t my_mx1508_set_speed(mx1508_t *dev,
                              int16_t left,
                              int16_t right)
{
    left  = left  * dev->max_percent / 100;
    right = right * dev->max_percent / 100;

    left  = apply_deadzone(left,  dev->deadzone_l);
    right = apply_deadzone(right, dev->deadzone_r);

    left  = slew(left,  dev->last_l, dev->max_step);
    right = slew(right, dev->last_r, dev->max_step);

    left  = clamp(left, -PWM_MAX, PWM_MAX);
    right = clamp(right, -PWM_MAX, PWM_MAX);

    drive(dev->ch1, dev->ch2, left);
    drive(dev->ch3, dev->ch4, right);

    dev->last_l = left;
    dev->last_r = right;

    return ESP_OK;
}

esp_err_t my_mx1508_coast(mx1508_t *dev)
{
    drive(dev->ch1, dev->ch2, 0);
    drive(dev->ch3, dev->ch4, 0);
    dev->last_l = dev->last_r = 0;
    return ESP_OK;
}
