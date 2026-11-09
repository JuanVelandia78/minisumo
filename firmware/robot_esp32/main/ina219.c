// INA219 por el bus I2C por software. La corriente se calcula directamente con el voltaje del
// shunt (registro 0x01, 10 uV por unidad) y INA_SHUNT_OHM, así no hace falta el registro de calibración.
#include "ina219.h"
#include "i2c_bus.h"
#include "config.h"
#include "esp_log.h"

static const char *TAG = "ina219";

#define REG_CONFIG  0x00
#define REG_SHUNT   0x01
#define REG_BUS     0x02

// BRNG=0 (bus hasta 16 V), PGA=/8 (±320 mV -> hasta 3,2 A con 0,1 ohm),
// BADC=12 bits, SADC=12 bits con promedio de 8 muestras (4,3 ms), modo continuo shunt + bus
#define INA_CONFIG  ((0u << 13) | (3u << 11) | (3u << 7) | (11u << 3) | 7u)

esp_err_t ina_init(void)
{
    i2c_bus_init();
    esp_err_t err = ESP_FAIL;
    for (int i = 0; i < 3 && err != ESP_OK; i++) err = i2c_bus_probe(INA_ADDR);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "No responde en 0x%02X: revisa SDA=GPIO%d, SCL=GPIO%d, VCC, A0 y A1",
                 INA_ADDR, PIN_I2C_SDA, PIN_I2C_SCL);
        return err;
    }
    err = i2c_bus_write16(INA_ADDR, REG_CONFIG, INA_CONFIG);
    if (err == ESP_OK) ESP_LOGI(TAG, "INA219 listo (shunt %.2f ohm, hasta 3,2 A)", (double)INA_SHUNT_OHM);
    return err;
}

esp_err_t ina_read(ina_sample_t *out)
{
    uint16_t sh, bus;
    esp_err_t err = i2c_bus_read16(INA_ADDR, REG_SHUNT, &sh);
    if (err == ESP_OK) err = i2c_bus_read16(INA_ADDR, REG_BUS, &bus);
    if (err != ESP_OK) return err;
    float v_sh = (int16_t)sh * 10e-6f;            // V en el shunt, con signo
    out->v_bus = (bus >> 3) * 4e-3f;               // bits 15..3, 4 mV por unidad
    out->v_bat = out->v_bus + v_sh;
    out->i = v_sh / INA_SHUNT_OHM;
    out->p = out->v_bat * out->i;
    return ESP_OK;
}

bool ina_config_ok(void)
{
    uint16_t v = 0;
    return i2c_bus_read16(INA_ADDR, REG_CONFIG, &v) == ESP_OK && v == INA_CONFIG;
}
