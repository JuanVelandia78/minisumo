// Filtros digitales del robot:
//   IIR (secciones de segundo orden) -> señales de la IMU
//   Media móvil                       -> velocidad de las ruedas (encoders)
#pragma once

#define MOVAVG_MAX   32
#define SOS_MAX      4

typedef struct {
    float buf[MOVAVG_MAX];
    float sum;
    int n, idx, count;
} movavg_t;

typedef struct {
    float b0, b1, b2, a1, a2;
    float z1, z2;
} biquad_t;

typedef struct {
    biquad_t s[SOS_MAX];
    int n;
} iir_t;

void movavg_init(movavg_t *f, int n);
float movavg_update(movavg_t *f, float x);

void iir_init(iir_t *f, const float (*sos)[5], int n);
float iir_update(iir_t *f, float x);
