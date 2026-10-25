#include <math.h>
#include "pid.h"

void pid_init(pid_ctrl_t *p, float kp, float ki, float kd, float kff, float duty_min, float trim, float out_max)
{
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
    p->kff = kff;
    p->duty_min = duty_min;
    p->trim = trim;
    p->out_max = out_max;
    pid_reset(p);
}

void pid_reset(pid_ctrl_t *p)
{
    p->integ = 0.0f;
    p->prev_meas = 0.0f;
}

static float clampf(float x, float lim)
{
    return x > lim ? lim : (x < -lim ? -lim : x);
}

float pid_update(pid_ctrl_t *p, float ref, float meas, float dt)
{
    if (fabsf(ref) < 1e-3f) {           // referencia cero: motor libre, sin esfuerzo acumulado
        pid_reset(p);
        p->prev_meas = meas;
        return 0.0f;
    }

    float e = ref - meas;
    float ff = p->kff * ref + copysignf(p->duty_min, ref);
    float d = -(meas - p->prev_meas) / dt;   // derivada sobre la medición (sin pico al cambiar ref)
    p->prev_meas = meas;

    float u_raw = ff + p->kp * e + p->integ + p->kd * d;
    // El PID solo corrige ±trim alrededor de la prealimentación y nunca invierte el sentido:
    // si el encoder mide velocidades falsas, la rueda sigue cerca de la velocidad pedida.
    float lo = ff - p->trim, hi = ff + p->trim;
    if (ref > 0) { if (lo < 0.0f) lo = 0.0f; if (hi > p->out_max) hi = p->out_max; }
    else         { if (hi > 0.0f) hi = 0.0f; if (lo < -p->out_max) lo = -p->out_max; }
    float u = u_raw < lo ? lo : (u_raw > hi ? hi : u_raw);

    // Anti-windup: integra solo si no está saturado o si el error ayuda a salir de la saturación
    if (u == u_raw || (u_raw > hi && e < 0) || (u_raw < lo && e > 0)) {
        p->integ = clampf(p->integ + p->ki * e * dt, p->trim);
    }
    return u;
}
