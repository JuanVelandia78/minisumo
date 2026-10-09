#include "motors.h"
#include "config.h"
#include "driver/ledc.h"
#include "driver/gpio.h"

#define CH_L LEDC_CHANNEL_0
#define CH_R LEDC_CHANNEL_1
#define DUTY_FULL ((1 << PWM_RES_BITS) - 1)

void motors_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_AIN1) | (1ULL << PIN_AIN2) | (1ULL << PIN_BIN1) | (1ULL << PIN_BIN2),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io);
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
    c.gpio_num = PIN_PWMA;
    c.channel = CH_L;
    ESP_ERROR_CHECK(ledc_channel_config(&c));
    c.gpio_num = PIN_PWMB;
    c.channel = CH_R;
    ESP_ERROR_CHECK(ledc_channel_config(&c));
}

static float clampf(float x, float lim)
{
    return x > lim ? lim : (x < -lim ? -lim : x);
}

static void set_one(ledc_channel_t ch, int in1, int in2, float duty)
{
    if (duty > 0.0f) {
        gpio_set_level(in1, 1);
        gpio_set_level(in2, 0);
    } else if (duty < 0.0f) {
        gpio_set_level(in1, 0);
        gpio_set_level(in2, 1);
        duty = -duty;
    } else {
        gpio_set_level(in1, 0);                 // libre (stop)
        gpio_set_level(in2, 0);
    }
    ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, (uint32_t)(duty * DUTY_FULL));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, ch);
}

void motors_set(float duty_l, float duty_r, float *applied_l, float *applied_r)
{
    duty_l = clampf(duty_l, PWM_MAX);
    duty_r = clampf(duty_r, PWM_MAX);
    set_one(CH_L, PIN_AIN1, PIN_AIN2, MOTOR_L_INVERT ? -duty_l : duty_l);
    set_one(CH_R, PIN_BIN1, PIN_BIN2, MOTOR_R_INVERT ? -duty_r : duty_r);
    if (applied_l) *applied_l = duty_l;
    if (applied_r) *applied_r = duty_r;
}

void motors_stop(void)
{
    motors_set(0.0f, 0.0f, NULL, NULL);
}
