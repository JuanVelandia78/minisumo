// Bus I2C por software (implementado en mpu6050.c) compartido por el MPU-6050 y el INA219.
// Usar solo desde una tarea a la vez (el lazo de control).
#pragma once
#include <stdint.h>
#include "esp_err.h"

void i2c_bus_init(void);
esp_err_t i2c_bus_probe(uint8_t addr);
// Registros de 16 bits, byte alto primero (formato del INA219)
esp_err_t i2c_bus_write16(uint8_t addr, uint8_t reg, uint16_t val);
esp_err_t i2c_bus_read16(uint8_t addr, uint8_t reg, uint16_t *val);
