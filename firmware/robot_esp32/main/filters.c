#include <string.h>
#include "filters.h"

void movavg_init(movavg_t *f, int n)
{
    memset(f, 0, sizeof(*f));
    f->n = (n < 1) ? 1 : (n > MOVAVG_MAX ? MOVAVG_MAX : n);
}

// Suma deslizante: O(1) por muestra sin importar N.
float movavg_update(movavg_t *f, float x)
{
    f->sum += x - f->buf[f->idx];
    f->buf[f->idx] = x;
    f->idx = (f->idx + 1) % f->n;
    if (f->count < f->n) f->count++;
    return f->sum / f->count;
}

void iir_init(iir_t *f, const float (*sos)[5], int n)
{
    memset(f, 0, sizeof(*f));
    f->n = (n > SOS_MAX) ? SOS_MAX : n;
    for (int i = 0; i < f->n; i++) {
        f->s[i].b0 = sos[i][0];
        f->s[i].b1 = sos[i][1];
        f->s[i].b2 = sos[i][2];
        f->s[i].a1 = sos[i][3];
        f->s[i].a2 = sos[i][4];
    }
}

// Forma directa II transpuesta, secciones en cascada.
float iir_update(iir_t *f, float x)
{
    for (int i = 0; i < f->n; i++) {
        biquad_t *s = &f->s[i];
        float y = s->b0 * x + s->z1;
        s->z1 = s->b1 * x - s->a1 * y + s->z2;
        s->z2 = s->b2 * x - s->a2 * y;
        x = y;
    }
    return x;
}
