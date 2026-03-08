#include "my_pid.h"

static float clamp(float value, float min, float max) {
    if (value > max) return max;
    if (value < min) return min;
    return value;
}

void pid_init(PIDController* pid,
              float Kp, float Ki, float Kd,
              float dt,
              float min_out, float max_out) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;

    pid->dt = dt;

    pid->setpoint = 0.0f;

    pid->integral = 0.0f;
    pid->prev_input = 0.0f;

    pid->last_d = 0.0f;
    pid->d_alpha = 0.7f;      
    pid->i_limit = 0.0f;      

    pid->output_min = min_out;
    pid->output_max = max_out;
}

void pid_set_limits(PIDController* pid, float i_limit) {
    pid->i_limit = i_limit;
}

void pid_set_d_filter(PIDController* pid, float alpha) {
    pid->d_alpha = clamp(alpha, 0.0f, 1.0f);
}

void pid_reset(PIDController* pid) {
    pid->integral = 0.0f;
    pid->prev_input = 0.0f;
    pid->last_d = 0.0f;
}

float pid_compute(PIDController* pid, float measurement) {
    float error = pid->setpoint - measurement;

    pid->integral += error * pid->dt;
    if (pid->i_limit > 0.0f) {
        pid->integral = clamp(pid->integral, -pid->i_limit, pid->i_limit);
    }

    float raw_d = -(measurement - pid->prev_input) / pid->dt;
    float d = pid->d_alpha * pid->last_d + (1.0f - pid->d_alpha) * raw_d;

    float output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * d;

    output = clamp(output, pid->output_min, pid->output_max);

    pid->prev_input = measurement;
    pid->last_d = d;

    return output;
}
