// Encoders de herradura de un canal (interrupción por flanco)
#pragma once
#include <stdint.h>

typedef struct {
    float omega;        // rad/s con signo (velocidad estimada por período entre flancos)
    float dtheta;       // rad girados desde la llamada anterior, con signo
    int32_t ticks;      // pulsos acumulados con signo
} enc_reading_t;

void enc_init(void);
// El sensor no detecta el sentido: se toma del ciclo útil aplicado (+1 / -1).
void enc_set_dir(int side, int dir);
// Llamar una vez por período de control. side: 0 = izquierdo, 1 = derecho.
void enc_update(int side, int64_t now_us, enc_reading_t *out);
void enc_reset(void);

#define ENC_LEFT  0
#define ENC_RIGHT 1
