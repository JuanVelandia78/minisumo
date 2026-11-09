// Encoders de cuadratura con el contador de pulsos PCNT del ESP32.
// Cada rueda usa una unidad PCNT con dos canales (C1 como flanco y C2 como nivel, y al revés):
// se cuentan los 4 flancos de cada ciclo y el sentido de giro sale del propio encoder.
#include <math.h>
#include "encoders.h"
#include "config.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "enc";

#define RAD_PER_COUNT  (2.0f * (float)M_PI / ENC_COUNTS_PER_REV)
#define PCNT_LIMIT     30000      // el conteo por hardware es de 16 bits; con accum_count se extiende a int

typedef struct {
    pcnt_unit_handle_t unit;
    int sign;                 // +1 o -1 (ENC_x_INVERT)
    int prev;                 // conteo en la llamada anterior
    int64_t prev_us;
} enc_chan_t;

static enc_chan_t s_ch[2];

static void setup_unit(enc_chan_t *c, int pin_a, int pin_b)
{
    pcnt_unit_config_t ucfg = {
        .high_limit = PCNT_LIMIT,
        .low_limit = -PCNT_LIMIT,
        .flags.accum_count = 1,   // al llegar al límite el driver suma y sigue contando
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&ucfg, &c->unit));
    pcnt_glitch_filter_config_t fcfg = {.max_glitch_ns = ENC_GLITCH_NS};
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(c->unit, &fcfg));

    pcnt_chan_config_t a = {.edge_gpio_num = pin_a, .level_gpio_num = pin_b};
    pcnt_chan_config_t b = {.edge_gpio_num = pin_b, .level_gpio_num = pin_a};
    pcnt_channel_handle_t ca, cb;
    ESP_ERROR_CHECK(pcnt_new_channel(c->unit, &a, &ca));
    ESP_ERROR_CHECK(pcnt_new_channel(c->unit, &b, &cb));
    // Decodificación x4 estándar
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(ca, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(ca, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(cb, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(cb, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(c->unit, PCNT_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(c->unit, -PCNT_LIMIT));

    // Pull-up interno por si el encoder tiene salida de colector abierto
    gpio_pullup_en(pin_a);
    gpio_pullup_en(pin_b);

    ESP_ERROR_CHECK(pcnt_unit_enable(c->unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(c->unit));
    ESP_ERROR_CHECK(pcnt_unit_start(c->unit));
}

void enc_init(void)
{
    s_ch[ENC_LEFT].sign = ENC_L_INVERT ? -1 : 1;
    s_ch[ENC_RIGHT].sign = ENC_R_INVERT ? -1 : 1;
    setup_unit(&s_ch[ENC_LEFT], PIN_ENC_L_C1, PIN_ENC_L_C2);
    setup_unit(&s_ch[ENC_RIGHT], PIN_ENC_R_C1, PIN_ENC_R_C2);
    ESP_LOGI(TAG, "Encoders de cuadratura: izq C1=GPIO%d C2=GPIO%d, der C1=GPIO%d C2=GPIO%d, %.0f pulsos/vuelta",
             PIN_ENC_L_C1, PIN_ENC_L_C2, PIN_ENC_R_C1, PIN_ENC_R_C2, (double)ENC_COUNTS_PER_REV);
}

void enc_reset(void)
{
    for (int i = 0; i < 2; i++) {
        pcnt_unit_clear_count(s_ch[i].unit);
        s_ch[i].prev = 0;
    }
}

void enc_update(int side, int64_t now_us, enc_reading_t *out)
{
    enc_chan_t *c = &s_ch[side];
    int count = 0;
    pcnt_unit_get_count(c->unit, &count);
    int dc = count - c->prev;
    c->prev = count;

    // Tiempo real entre llamadas (normalmente CONTROL_DT); si es la primera vez, se usa el nominal
    float dt = (c->prev_us > 0 && now_us > c->prev_us) ? (now_us - c->prev_us) * 1e-6f : CONTROL_DT;
    c->prev_us = now_us;

    float dth = c->sign * dc * RAD_PER_COUNT;
    out->dtheta = dth;
    out->omega = dth / dt;
    out->ticks = c->sign * count;
}
