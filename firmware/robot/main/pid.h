// PID de velocidad con prealimentación, compensación de zona muerta y anti-windup
#pragma once

typedef struct {
    float kp, ki, kd, kff, duty_min, trim;
    float out_max;
    float integ;
    float prev_meas;
} pid_ctrl_t;

void pid_init(pid_ctrl_t *p, float kp, float ki, float kd, float kff, float duty_min, float trim, float out_max);
void pid_reset(pid_ctrl_t *p);
// ref y meas en rad/s, devuelve ciclo útil en [-out_max, out_max]
float pid_update(pid_ctrl_t *p, float ref, float meas, float dt);
