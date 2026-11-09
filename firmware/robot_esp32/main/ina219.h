// INA219: voltaje y corriente de la batería 2S (shunt de 0,1 ohm del módulo)
#pragma once
#include <stdbool.h>
#include "esp_err.h"

typedef struct {
    float v_bat;    // V en Vin+ (batería) = bus (Vin-) + caída en el shunt
    float v_bus;    // V en Vin- (lo que le llega al DRV8833)
    float i;        // A (positiva = la batería entrega)
    float p;        // W entregados por la batería
} ina_sample_t;

esp_err_t ina_init(void);
esp_err_t ina_read(ina_sample_t *out);
// true si el registro de configuración tiene lo que escribió ina_init (el módulo responde bien)
bool ina_config_ok(void);
