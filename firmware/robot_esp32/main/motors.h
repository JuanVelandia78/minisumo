// Motores con TB6612FNG: PWM por LEDC y dos pines de dirección por motor
#pragma once

void motors_init(void);
// duty en [-1, 1]; se limita a ±PWM_MAX. Devuelve el ciclo útil realmente aplicado.
void motors_set(float duty_l, float duty_r, float *applied_l, float *applied_r);
void motors_stop(void);
