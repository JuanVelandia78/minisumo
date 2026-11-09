#include <math.h>
#include "selftest.h"
#include "config.h"
#include "mpu6050.h"
#include "ina219.h"
#include "motors.h"
#include "encoders.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "diagnostico";
static int s_fails;

#define OK(fmt, ...)   ESP_LOGI(TAG, "[ OK  ] " fmt, ##__VA_ARGS__)
#define FAIL(fmt, ...) do { ESP_LOGE(TAG, "[FALLA] " fmt, ##__VA_ARGS__); s_fails++; } while (0)
#define WARN(fmt, ...) ESP_LOGW(TAG, "[AVISO] " fmt, ##__VA_ARGS__)
#define SKIP(fmt, ...) ESP_LOGW(TAG, "[ --  ] " fmt, ##__VA_ARGS__)

// ---------------------------------------------------------------------------
// Bus I2C: las dos líneas deben estar en alto en reposo y no estar unidas entre sí
// ---------------------------------------------------------------------------
static bool test_i2c_lines(void)
{
    int sda = gpio_get_level(PIN_I2C_SDA), scl = gpio_get_level(PIN_I2C_SCL);
    if (!sda || !scl) {
        FAIL("Bus I2C: %s%s%s en bajo en reposo -> corto a GND, módulo sin alimentar que jala la línea, "
             "o soldadura unida (SDA=GPIO%d, SCL=GPIO%d)",
             !sda ? "SDA" : "", (!sda && !scl) ? " y " : "", !scl ? "SCL" : "", PIN_I2C_SDA, PIN_I2C_SCL);
        return false;
    }
    // SCL en bajo un momento: si SDA también baja, las dos pistas están en corto
    gpio_set_level(PIN_I2C_SCL, 0);
    esp_rom_delay_us(20);
    int sda_follow = gpio_get_level(PIN_I2C_SDA);
    gpio_set_level(PIN_I2C_SCL, 1);
    esp_rom_delay_us(20);
    if (!sda_follow) {
        FAIL("Bus I2C: SDA y SCL están en corto entre sí (GPIO%d y GPIO%d)", PIN_I2C_SDA, PIN_I2C_SCL);
        return false;
    }
    OK("Bus I2C: SDA=GPIO%d y SCL=GPIO%d en alto y separadas", PIN_I2C_SDA, PIN_I2C_SCL);
    return true;
}

// ---------------------------------------------------------------------------
// MPU-6050: responde, los datos cambian y la gravedad mide ~1 g
// ---------------------------------------------------------------------------
static void test_mpu(bool imu_ok)
{
    if (!imu_ok) {
        FAIL("MPU-6050 (0x%02X): no responde -> revisa VCC=3V3, GND, SDA=GPIO%d, SCL=GPIO%d y que AD0 esté al aire o a GND",
             MPU_ADDR, PIN_I2C_SDA, PIN_I2C_SCL);
        return;
    }
    imu_sample_t s, first = {0};
    float an = 0, gmax = 0;
    int ok = 0;
    bool changes = false;
    for (int i = 0; i < 20; i++) {
        if (mpu_read(&s) == ESP_OK) {
            if (ok == 0) first = s;
            else if (s.ax != first.ax || s.ay != first.ay || s.az != first.az || s.gx != first.gx) changes = true;
            an += sqrtf(s.ax * s.ax + s.ay * s.ay + s.az * s.az);
            float g = fmaxf(fabsf(s.gx), fmaxf(fabsf(s.gy), fabsf(s.gz)));
            if (g > gmax) gmax = g;
            ok++;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    if (ok < 18) {
        FAIL("MPU-6050: falló la lectura %d de 20 veces -> conexión floja o ruido en SDA/SCL", 20 - ok);
        return;
    }
    an /= ok;
    if (!changes) {
        FAIL("MPU-6050: los datos no cambian (todas las lecturas iguales) -> el sensor está trabado o es un clon dañado");
        return;
    }
    if (an < 0.7f || an > 1.3f) {
        FAIL("MPU-6050: la gravedad mide %.2f g (se esperaba ~1,00 g con el robot quieto) -> sensor dañado", an);
        return;
    }
    OK("MPU-6050 (0x%02X): responde, gravedad %.2f g, giro máx. %.1f °/s", MPU_ADDR, an, gmax);
    if (gmax > 25.0f) WARN("MPU-6050: giro alto (%.0f °/s) sin moverlo: ¿se movió el robot durante la prueba?", gmax);
}

// ---------------------------------------------------------------------------
// INA219 y batería 2S
// ---------------------------------------------------------------------------
static bool read_ina_avg(int n, ina_sample_t *out)
{
    ina_sample_t s, acc = {0};
    int ok = 0;
    for (int i = 0; i < n; i++) {
        if (ina_read(&s) == ESP_OK) {
            acc.v_bat += s.v_bat; acc.v_bus += s.v_bus; acc.i += s.i; acc.p += s.p;
            ok++;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    if (ok == 0) return false;
    out->v_bat = acc.v_bat / ok; out->v_bus = acc.v_bus / ok; out->i = acc.i / ok; out->p = acc.p / ok;
    return true;
}

// Devuelve true si se puede usar el INA para probar el puente H (responde y hay batería)
static bool test_ina(bool ina_ok, ina_sample_t *rest)
{
    if (!ina_ok) {
        FAIL("INA219 (0x%02X): no responde -> revisa VCC=3V3, GND, SDA=GPIO%d, SCL=GPIO%d y que A0/A1 no tengan puente soldado",
             INA_ADDR, PIN_I2C_SDA, PIN_I2C_SCL);
        return false;
    }
    if (!ina_config_ok()) {
        FAIL("INA219: responde pero no guarda la configuración -> módulo dañado o ruido en el bus");
        return false;
    }
    if (!read_ina_avg(10, rest)) {
        FAIL("INA219: falló la lectura de voltaje y corriente");
        return false;
    }
    OK("INA219 (0x%02X): responde y guarda la configuración", INA_ADDR);

    if (rest->v_bat < BAT_ABSENT_V) {
        FAIL("Batería 2S: %.2f V -> batería sin conectar, interruptor apagado o J8 (Vin+) sin soldar", rest->v_bat);
        return false;
    }
    if (rest->v_bat < 6.0f) FAIL("Batería 2S: %.2f V, demasiado baja (mínimo 6,0 V) -> cárgala", rest->v_bat);
    else if (rest->v_bat > 9.0f) FAIL("Batería 2S: %.2f V, demasiado alta para una 2S -> ¿conectaste una 3S? El DRV8833 aguanta hasta 10,8 V", rest->v_bat);
    else if (rest->v_bat < BAT_LOW_V) WARN("Batería 2S: %.2f V, baja: cárgala pronto", rest->v_bat);
    else OK("Batería 2S: %.2f V (al DRV8833 llegan %.2f V)", rest->v_bat, rest->v_bus);

    if (rest->v_bus < BAT_ABSENT_V) {
        FAIL("Vin- del INA219 a 0 V con la batería conectada -> pista Vin- / VM del DRV8833 abierta");
        return false;
    }
    if (rest->i < -0.02f) {
        FAIL("Corriente negativa en reposo (%.3f A) -> Vin+ y Vin- del INA219 están invertidos", rest->i);
        return false;
    }
    if (rest->i > 0.20f) {
        FAIL("Consumo en reposo de %.3f A con los motores apagados -> posible corto en VM, el DRV8833 o los motores", rest->i);
        return false;
    }
    OK("Consumo en reposo del lado de los motores: %.1f mA", rest->i * 1000.0f);
    return true;
}

// ---------------------------------------------------------------------------
// Puente H: pulso corto y suave a cada motor y en cada sentido; el INA219 debe ver corriente
// ---------------------------------------------------------------------------
static float pulse_current(float dl, float dr, int32_t *dticks)
{
    enc_reading_t e0, e1;
    int side = (dl != 0.0f) ? ENC_LEFT : ENC_RIGHT;
    enc_update(side, esp_timer_get_time(), &e0);
    motors_set(dl, dr, NULL, NULL);
    vTaskDelay(pdMS_TO_TICKS(SELFTEST_PULSE_MS / 2));     // deja pasar el transitorio
    ina_sample_t a = {0}, b = {0};
    ina_read(&a);
    vTaskDelay(pdMS_TO_TICKS(SELFTEST_PULSE_MS / 2));
    ina_read(&b);
    motors_stop();
    vTaskDelay(pdMS_TO_TICKS(150));                       // la rueda se detiene
    enc_update(side, esp_timer_get_time(), &e1);
    *dticks = e1.ticks - e0.ticks;
    return (a.i + b.i) * 0.5f;
}

static void test_motor(const char *name, const char *pins, bool left, float i_rest)
{
    const float d = SELFTEST_DUTY;
    int32_t tf, tr;
    float i_f = pulse_current(left ? d : 0, left ? 0 : d, &tf) - i_rest;
    float i_r = pulse_current(left ? -d : 0, left ? 0 : -d, &tr) - i_rest;
    bool f = i_f > SELFTEST_I_MIN, r = i_r > SELFTEST_I_MIN;
    if (f && r) {
        OK("Puente H motor %s: adelante %.0f mA, atrás %.0f mA", name, i_f * 1000.0f, i_r * 1000.0f);
    } else if (!f && !r) {
        FAIL("Puente H motor %s: no circula corriente en ningún sentido (%.0f / %.0f mA) -> motor sin conectar en J6, "
             "cable roto o salida del DRV8833 dañada", name, i_f * 1000.0f, i_r * 1000.0f);
    } else {
        FAIL("Puente H motor %s: solo funciona %s (%.0f / %.0f mA) -> revisa %s, su resistencia de 220 ohm y la soldadura en J2",
             name, f ? "hacia adelante" : "hacia atrás", i_f * 1000.0f, i_r * 1000.0f, pins);
    }
    if (tf != 0 || tr != 0)
        ESP_LOGI(TAG, "        encoder %s durante el pulso: %+ld / %+ld ticks", name, (long)tf, (long)tr);
}

static void test_hbridge(bool can, float i_rest)
{
    if (!SELFTEST_MOTORS) {
        SKIP("Puente H: prueba desactivada (SELFTEST_MOTORS = 0)");
        return;
    }
    if (!can) {
        SKIP("Puente H: no se puede probar sin el INA219 funcionando y la batería conectada");
        return;
    }
    int before = s_fails;
    test_motor("izquierdo", "AIN1 (GPIO33) / AIN2 (GPIO32)", true, i_rest);
    test_motor("derecho", "BIN1 (GPIO14) / BIN2 (GPIO13)", false, i_rest);
    if (s_fails - before == 2)
        WARN("Fallan los dos motores: lo más probable es STBY (GPIO26 -> J2), la alimentación VM o GND del DRV8833");
}

int selftest_run(bool imu_ok, bool ina_ok)
{
    s_fails = 0;
    motors_stop();
    ESP_LOGI(TAG, "================ AUTODIAGNÓSTICO DE LA PCB ================");
    bool bus = test_i2c_lines();
    test_mpu(imu_ok && bus);
    ina_sample_t rest = {0};
    bool can = test_ina(ina_ok && bus, &rest);
    test_hbridge(can, rest.i);
    motors_stop();
    SKIP("Encoders: gira cada rueda a mano y mira los ticks en el mensaje de estado (cada 2 s)");
    if (s_fails == 0) ESP_LOGI(TAG, "================ RESULTADO: TODO BIEN ================");
    else ESP_LOGE(TAG, "================ RESULTADO: %d FALLA(S), revisa arriba ================", s_fails);
    return s_fails;
}

void selftest_blink(int fails)
{
    gpio_reset_pin(PIN_LED);
    gpio_set_direction(PIN_LED, GPIO_MODE_OUTPUT);
    int n = fails ? 10 : 3, ms = fails ? 100 : 300;
    for (int i = 0; i < n; i++) {
        gpio_set_level(PIN_LED, 1);
        vTaskDelay(pdMS_TO_TICKS(ms));
        gpio_set_level(PIN_LED, 0);
        vTaskDelay(pdMS_TO_TICKS(ms));
    }
}
