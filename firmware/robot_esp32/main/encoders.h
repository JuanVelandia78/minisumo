// Encoders de cuadratura (C1/C2) de cada motor, contados por el periférico PCNT en x4
#pragma once
#include <stdint.h>

typedef struct {
    float omega;        // rad/s con signo (pulsos en el periodo de control / CONTROL_DT)
    float dtheta;       // rad girados desde la llamada anterior, con signo
    int32_t ticks;      // pulsos acumulados con signo
} enc_reading_t;

void enc_init(void);
// Llamar una vez por período de control. side: 0 = izquierdo, 1 = derecho.
void enc_update(int side, int64_t now_us, enc_reading_t *out);
void enc_reset(void);

#define ENC_LEFT  0
#define ENC_RIGHT 1
