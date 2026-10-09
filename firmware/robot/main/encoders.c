#include <math.h>
#include "encoders.h"
#include "config.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_attr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define EDGES_PER_REV  (ENC_SLOTS * (ENC_BOTH_EDGES ? 2 : 1))
#define RAD_PER_EDGE   (2.0f * (float)M_PI / EDGES_PER_REV)

typedef struct {
    // escritos por la ISR
    volatile uint32_t edges;
    volatile int64_t last_edge_us;
    volatile int64_t last_interval_us;   // 0 si la rueda venía detenida
    // usados por el lazo de control
    uint32_t prev_edges;
    int64_t prev_edge_us;
    int dir;
    float omega;
    int32_t ticks;
} enc_chan_t;

static enc_chan_t s_ch[2];
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

// Acepta un flanco en el instante t si pasa el filtro de rebotes (llamar dentro de s_mux)
static inline void IRAM_ATTR accept_edge(enc_chan_t *c, int64_t t)
{
    int64_t dt = t - c->last_edge_us;
    // Umbral adaptativo: un flanco real no llega mucho antes que el intervalo anterior.
    // Así se descartan rebotes del comparador y ruido de los motores en la línea.
    int64_t min_dt = ENC_DEBOUNCE_US;
    if (c->last_interval_us > 0) {
        int64_t adapt = c->last_interval_us * ENC_GLITCH_PCT / 100;
        if (adapt > ENC_GLITCH_MAX_US) adapt = ENC_GLITCH_MAX_US;
        if (adapt > min_dt) min_dt = adapt;
    }
    if (dt >= min_dt) {
        c->last_interval_us = (dt < ENC_STOP_TIMEOUT_US) ? dt : 0;
        c->last_edge_us = t;
        c->edges++;
    }
}

// Salida D0 del LM393: interrupción por flanco
static void IRAM_ATTR enc_isr(void *arg)
{
    int64_t t = esp_timer_get_time();
    portENTER_CRITICAL_ISR(&s_mux);
    accept_edge((enc_chan_t *)arg, t);
    portEXIT_CRITICAL_ISR(&s_mux);
}

static void enc_hw_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_ENC_L) | (1ULL << PIN_ENC_R),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,       // el módulo ya trae resistencia pull-up
        .intr_type = ENC_BOTH_EDGES ? GPIO_INTR_ANYEDGE : GPIO_INTR_POSEDGE,
    };
    gpio_config(&io);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(PIN_ENC_L, enc_isr, &s_ch[ENC_LEFT]);
    gpio_isr_handler_add(PIN_ENC_R, enc_isr, &s_ch[ENC_RIGHT]);
}


void enc_init(void)
{
    for (int i = 0; i < 2; i++) s_ch[i].dir = 1;
    enc_hw_init();
}

void enc_set_dir(int side, int dir)
{
    if (dir != 0) s_ch[side].dir = dir > 0 ? 1 : -1;
}

void enc_reset(void)
{
    for (int i = 0; i < 2; i++) s_ch[i].ticks = 0;
}

void enc_update(int side, int64_t now_us, enc_reading_t *out)
{
    enc_chan_t *c = &s_ch[side];
    portENTER_CRITICAL(&s_mux);
    uint32_t edges = c->edges;
    int64_t last = c->last_edge_us;
    portEXIT_CRITICAL(&s_mux);

    uint32_t de = edges - c->prev_edges;
    float speed;   // magnitud en rad/s

    if (de > 0) {
        // Velocidad por período: ángulo de los flancos nuevos / tiempo entre el último flanco
        // anterior y el último flanco actual. Mejor resolución que contar en una ventana fija.
        int64_t span = last - c->prev_edge_us;
        if (c->prev_edge_us == 0 || span <= 0 || span > ENC_STOP_TIMEOUT_US) {
            speed = 0.0f;   // primer flanco tras estar detenido: aún no hay período válido
        } else {
            speed = de * RAD_PER_EDGE / (span * 1e-6f);
            // Una velocidad imposible para el motor es ruido en la línea D0: se conserva la anterior
            if (speed > ENC_OMEGA_PHYS_MAX) speed = fabsf(c->omega);
        }
        c->prev_edge_us = last;
        c->prev_edges = edges;
    } else {
        // Sin flancos nuevos: la velocidad no puede ser mayor que un pulso en el tiempo transcurrido.
        int64_t since = now_us - c->prev_edge_us;
        float prev = fabsf(c->omega);
        if (c->prev_edge_us == 0 || since > ENC_STOP_TIMEOUT_US) {
            speed = 0.0f;
        } else {
            float bound = RAD_PER_EDGE / (since * 1e-6f);
            speed = prev < bound ? prev : bound;
        }
    }

    c->omega = c->dir * speed;
    c->ticks += c->dir * (int32_t)de;

    out->omega = c->omega;
    out->dtheta = c->dir * (int32_t)de * RAD_PER_EDGE;
    out->ticks = c->ticks;
}
