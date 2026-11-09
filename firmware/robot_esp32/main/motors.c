#include "motors.h"
#include "config.h"
#include "driver/ledc.h"
#include "driver/gpio.h"

// Un canal LEDC por cada pin IN del DRV8833 (4 en total, mismo temporizador de 20 kHz)
#define CH_AIN1 LEDC_CHANNEL_0
#define CH_AIN2 LEDC_CHANNEL_1
#define CH_BIN1 LEDC_CHANNEL_2
#define CH_BIN2 LEDC_CHANNEL_3
#define DUTY_FULL (1u << PWM_RES_BITS)     // en el LEDC, 2^bits = salida siempre en alto

void motors_init(void)
{
    // Primero el driver dormido y los IN en bajo: al arrancar el ESP los pines pueden dar pulsos
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_STBY) | (1ULL << PIN_AIN1) | (1ULL << PIN_AIN2) |
                        (1ULL << PIN_BIN1) | (1ULL << PIN_BIN2),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io);
    gpio_set_level(PIN_STBY, 0);
    gpio_set_level(PIN_AIN1, 0);
    gpio_set_level(PIN_AIN2, 0);
    gpio_set_level(PIN_BIN1, 0);
    gpio_set_level(PIN_BIN2, 0);

    ledc_timer_config_t t = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = PWM_RES_BITS,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&t));

    ledc_channel_config_t c = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
    };
    const int pins[4] = {PIN_AIN1, PIN_AIN2, PIN_BIN1, PIN_BIN2};
    const ledc_channel_t chs[4] = {CH_AIN1, CH_AIN2, CH_BIN1, CH_BIN2};
    for (int i = 0; i < 4; i++) {
        c.gpio_num = pins[i];
        c.channel = chs[i];
        ESP_ERROR_CHECK(ledc_channel_config(&c));
    }

    gpio_set_level(PIN_STBY, 1);            // DRV8833 activo (los IN ya están en bajo: motores libres)
}

static float clampf(float x, float lim)
{
    return x > lim ? lim : (x < -lim ? -lim : x);
}

static void set_ch(ledc_channel_t ch, uint32_t duty)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, ch);
}

// Tabla del DRV8833 (xIN1, xIN2): 1,0 adelante | 0,1 atrás | 0,0 libre | 1,1 freno
//   Slow decay: un IN fijo en alto y el otro con PWM invertido -> alterna entre avanzar y frenar.
//   Fast decay: un IN con PWM y el otro en bajo -> alterna entre avanzar y quedar libre.
static void set_one(ledc_channel_t ch1, ledc_channel_t ch2, float duty)
{
    float d = duty < 0.0f ? -duty : duty;
    uint32_t on = (uint32_t)(d * DUTY_FULL);
    uint32_t pwm_hi, pwm_lo;
#if MOTOR_SLOW_DECAY
    pwm_hi = DUTY_FULL;                     // pin que queda siempre en alto
    pwm_lo = DUTY_FULL - on;                // en bajo durante el tiempo útil
#else
    pwm_hi = on;
    pwm_lo = 0;
#endif
    if (duty > 0.0f) {
        set_ch(ch1, pwm_hi);
        set_ch(ch2, pwm_lo);
    } else if (duty < 0.0f) {
        set_ch(ch1, pwm_lo);
        set_ch(ch2, pwm_hi);
    } else {
        set_ch(ch1, 0);                     // libre (stop)
        set_ch(ch2, 0);
    }
}

void motors_set(float duty_l, float duty_r, float *applied_l, float *applied_r)
{
    duty_l = clampf(duty_l, PWM_MAX);
    duty_r = clampf(duty_r, PWM_MAX);
    set_one(CH_AIN1, CH_AIN2, MOTOR_L_INVERT ? -duty_l : duty_l);
    set_one(CH_BIN1, CH_BIN2, MOTOR_R_INVERT ? -duty_r : duty_r);
    if (applied_l) *applied_l = duty_l;
    if (applied_r) *applied_r = duty_r;
}

void motors_stop(void)
{
    motors_set(0.0f, 0.0f, NULL, NULL);
}
