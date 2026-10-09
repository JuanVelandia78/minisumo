// MPU-6050 por I2C por software (bit-bang), igual que en el robot: cada espera tiene
// límite y el bus se recupera solo si una lectura falla.
#include <stdbool.h>
#include "mpu6050.h"
#include "config.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "mpu";

#define REG_SMPLRT_DIV   0x19
#define REG_CONFIG       0x1A
#define REG_GYRO_CONFIG  0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B
#define REG_PWR_MGMT_1   0x6B
#define REG_WHO_AM_I     0x75

#define ACCEL_LSB_PER_G   8192.0f   // ±4 g
#define GYRO_LSB_PER_DPS  65.5f     // ±500 °/s

static float s_gbias[3];

// ---------------------------------------------------------------------------
// I2C por software (~150 kHz)
// ---------------------------------------------------------------------------
#define I2C_HALF_US        3       // medio período de SCL
#define I2C_STRETCH_MAX_US 200     // espera máxima a que el esclavo libere SCL

static inline void dly(void) { esp_rom_delay_us(I2C_HALF_US); }
static inline void sda_hi(void) { gpio_set_level(PIN_I2C_SDA, 1); }   // open-drain: suelta la línea
static inline void sda_lo(void) { gpio_set_level(PIN_I2C_SDA, 0); }
static inline void scl_lo(void) { gpio_set_level(PIN_I2C_SCL, 0); }
static inline int sda_rd(void) { return gpio_get_level(PIN_I2C_SDA); }

static bool scl_hi(void)
{
    gpio_set_level(PIN_I2C_SCL, 1);
    for (int t = 0; !gpio_get_level(PIN_I2C_SCL); t++) {   // clock stretching con límite
        if (t > I2C_STRETCH_MAX_US) return false;
        esp_rom_delay_us(1);
    }
    return true;
}

static bool i2c_start(void)
{
    sda_hi();
    dly();
    if (!scl_hi()) return false;
    dly();
    if (!sda_rd()) return false;   // alguien retiene SDA
    sda_lo();
    dly();
    scl_lo();
    dly();
    return true;
}

static bool i2c_stop(void)
{
    sda_lo();
    dly();
    if (!scl_hi()) return false;
    dly();
    sda_hi();
    dly();
    return sda_rd();
}

static bool i2c_write_byte(uint8_t b)
{
    for (int i = 0; i < 8; i++) {
        if (b & 0x80) sda_hi(); else sda_lo();
        b <<= 1;
        dly();
        if (!scl_hi()) return false;
        dly();
        scl_lo();
    }
    sda_hi();                       // el esclavo responde ACK bajando SDA
    dly();
    if (!scl_hi()) return false;
    bool ack = !sda_rd();
    dly();
    scl_lo();
    return ack;
}

static bool i2c_read_byte(uint8_t *out, bool ack)
{
    uint8_t v = 0;
    sda_hi();
    for (int i = 0; i < 8; i++) {
        dly();
        if (!scl_hi()) return false;
        v = (v << 1) | (sda_rd() ? 1 : 0);
        dly();
        scl_lo();
    }
    if (ack) sda_lo(); else sda_hi();
    dly();
    if (!scl_hi()) return false;
    dly();
    scl_lo();
    sda_hi();
    *out = v;
    return true;
}

// Libera un esclavo que quedó a mitad de un byte: hasta 9 pulsos de SCL y un STOP.
static void i2c_recover(void)
{
    sda_hi();
    for (int i = 0; i < 9 && !sda_rd(); i++) {
        scl_lo();
        dly();
        gpio_set_level(PIN_I2C_SCL, 1);
        dly();
    }
    scl_lo();
    dly();
    sda_lo();
    dly();
    gpio_set_level(PIN_I2C_SCL, 1);
    dly();
    sda_hi();
    dly();
}

static esp_err_t wr(uint8_t reg, uint8_t val)
{
    bool ok = i2c_start() && i2c_write_byte(MPU_ADDR << 1) && i2c_write_byte(reg) && i2c_write_byte(val);
    ok = i2c_stop() && ok;
    if (!ok) i2c_recover();
    return ok ? ESP_OK : ESP_FAIL;
}

static esp_err_t rd(uint8_t reg, uint8_t *buf, size_t n)
{
    bool ok = i2c_start() && i2c_write_byte(MPU_ADDR << 1) && i2c_write_byte(reg) &&
              i2c_start() && i2c_write_byte((MPU_ADDR << 1) | 1);     // inicio repetido
    for (size_t i = 0; ok && i < n; i++) {
        ok = i2c_read_byte(&buf[i], i + 1 < n);                      // NACK en el último byte
    }
    ok = i2c_stop() && ok;
    if (!ok) i2c_recover();
    return ok ? ESP_OK : ESP_FAIL;
}

static esp_err_t probe(void)
{
    bool ok = i2c_start() && i2c_write_byte(MPU_ADDR << 1);
    ok = i2c_stop() && ok;
    if (!ok) i2c_recover();
    return ok ? ESP_OK : ESP_FAIL;
}

esp_err_t mpu_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_I2C_SDA) | (1ULL << PIN_I2C_SCL),
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&io);
    sda_hi();
    gpio_set_level(PIN_I2C_SCL, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
    i2c_recover();

    esp_err_t err = ESP_FAIL;
    for (int i = 0; i < 5 && err != ESP_OK; i++) {
        err = probe();
        if (err != ESP_OK) vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "No responde en 0x%02X: revisa SDA=GPIO%d, SCL=GPIO%d, VCC y AD0", MPU_ADDR, PIN_I2C_SDA, PIN_I2C_SCL);
        return err;
    }

    wr(REG_PWR_MGMT_1, 0x80);                       // reset
    vTaskDelay(pdMS_TO_TICKS(100));
    err = wr(REG_PWR_MGMT_1, 0x01);                 // despierta, reloj PLL del giroscopio X
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(10));

    uint8_t who = 0;
    rd(REG_WHO_AM_I, &who, 1);
    if (who != 0x68) {
        ESP_LOGW(TAG, "WHO_AM_I = 0x%02X (se esperaba 0x68). Puede ser un clon; se continúa.", who);
    }

    wr(REG_CONFIG, MPU_DLPF_CFG);                   // con DLPF activo la base es 1 kHz
    wr(REG_SMPLRT_DIV, (1000 / SAMPLE_HZ) - 1);    // 1 kHz / 10 = 100 Hz
    wr(REG_GYRO_CONFIG, 0x08);                      // ±500 °/s
    err = wr(REG_ACCEL_CONFIG, 0x08);               // ±4 g
    ESP_LOGI(TAG, "MPU-6050 listo (DLPF=%d, %d Hz)", MPU_DLPF_CFG, SAMPLE_HZ);
    return err;
}

static inline int16_t be16(const uint8_t *p)
{
    return (int16_t)((p[0] << 8) | p[1]);
}

esp_err_t mpu_read(imu_sample_t *out)
{
    uint8_t b[14];
    esp_err_t err = rd(REG_ACCEL_XOUT_H, b, sizeof(b));
    if (err != ESP_OK) return err;
    out->ax = be16(&b[0]) / ACCEL_LSB_PER_G;
    out->ay = be16(&b[2]) / ACCEL_LSB_PER_G;
    out->az = be16(&b[4]) / ACCEL_LSB_PER_G;
    // b[6..7] es temperatura
    out->gx = be16(&b[8]) / GYRO_LSB_PER_DPS - s_gbias[0];
    out->gy = be16(&b[10]) / GYRO_LSB_PER_DPS - s_gbias[1];
    out->gz = be16(&b[12]) / GYRO_LSB_PER_DPS - s_gbias[2];
    return ESP_OK;
}

void mpu_track_bias(const imu_sample_t *s, float alpha)
{
    // El bias cambia con la temperatura del chip: con el robot quieto, lo que mide el giroscopio
    // es solo bias residual, así que se suma poco a poco al bias guardado.
    s_gbias[0] += alpha * s->gx;
    s_gbias[1] += alpha * s->gy;
    s_gbias[2] += alpha * s->gz;
}

esp_err_t mpu_calibrate_gyro(int n)
{
    float sum[3] = {0};
    int ok = 0;
    s_gbias[0] = s_gbias[1] = s_gbias[2] = 0.0f;
    for (int i = 0; i < n; i++) {
        imu_sample_t s;
        if (mpu_read(&s) == ESP_OK) {
            sum[0] += s.gx;
            sum[1] += s.gy;
            sum[2] += s.gz;
            ok++;
        }
        vTaskDelay(pdMS_TO_TICKS(1000 / SAMPLE_HZ));
    }
    if (ok == 0) return ESP_FAIL;
    for (int k = 0; k < 3; k++) s_gbias[k] = sum[k] / ok;
    ESP_LOGI(TAG, "Bias giroscopio: %.3f %.3f %.3f °/s (%d muestras)", s_gbias[0], s_gbias[1], s_gbias[2], ok);
    return ESP_OK;
}
