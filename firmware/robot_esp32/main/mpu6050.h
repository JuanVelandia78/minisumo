// MPU-6050 por I2C (driver i2c_master de ESP-IDF 5.x)
#pragma once
#include "esp_err.h"

typedef struct {
    float ax, ay, az;   // g
    float gx, gy, gz;   // grados/s, sin bias
} imu_sample_t;

esp_err_t mpu_init(void);
esp_err_t mpu_read(imu_sample_t *out);
// Promedia n lecturas con el robot quieto y guarda el bias del giroscopio.
esp_err_t mpu_calibrate_gyro(int n);
// Ajuste lento del bias del giroscopio con el robot quieto (s es una lectura ya corregida)
void mpu_track_bias(const imu_sample_t *s, float alpha);
