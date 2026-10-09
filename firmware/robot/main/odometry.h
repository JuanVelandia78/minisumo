// Estimación de posición: odometría de encoders + giroscopio Z para el rumbo
#pragma once
#include <stdbool.h>

typedef struct {
    float x, y;        // m
    float theta;       // rad, rumbo fusionado
    float theta_enc;   // rad, solo encoders
    float theta_gyro;  // rad, solo giroscopio
} odom_t;

void odom_reset(odom_t *o);
// dth_l, dth_r: rad girados por cada rueda; gz: rad/s; moving: hay referencia o giro de ruedas
void odom_update(odom_t *o, float dth_l, float dth_r, float gz, float dt, bool moving);
