// Motores con DRV8833: el PWM (LEDC) se aplica directo en los pines IN1/IN2 de cada puente
// y el pin STBY (nSLEEP) habilita el driver.
#pragma once

void motors_init(void);
// duty en [-1, 1]; se limita a ±PWM_MAX. Devuelve el ciclo útil realmente aplicado.
void motors_set(float duty_l, float duty_r, float *applied_l, float *applied_r);
void motors_stop(void);
