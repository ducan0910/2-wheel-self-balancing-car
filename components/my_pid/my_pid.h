typedef struct {
    float Kp;
    float Ki;
    float Kd;

    float setpoint;
    float integral;
    float prev_input;

    float last_d;      
    float d_alpha;    
    float i_limit;     

    float output_min;
    float output_max;

    float dt;
} PIDController;
void pid_init(PIDController* pid,
              float Kp, float Ki, float Kd,
              float dt,
              float min_out, float max_out);
void pid_set_limits(PIDController* pid, float i_limit);
void pid_set_d_filter(PIDController* pid, float alpha);
void pid_reset(PIDController* pid);

float pid_compute(PIDController* pid, float measurement);