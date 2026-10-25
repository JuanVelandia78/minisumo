// Configuración del guante de teleoperación (ESP32 DevKit + MPU-6050)
#pragma once

// ---------------------------------------------------------------------------
// WiFi y red (la misma red del robot y del PC con ROS2)
// ---------------------------------------------------------------------------
#include "wifi_secrets.h"           // WIFI_SSID y WIFI_PASS (no se sube al repositorio, ver wifi_secrets.example.h)
#define GLOVE_UDP_PORT      8890               // el guante envía a este puerto; lo escucha glove_teleop
#define PC_TIMEOUT_US       3000000            // sin aviso del PC en 3 s -> vuelve a enviar por broadcast

// ---------------------------------------------------------------------------
// Pines (ESP32 DevKit)
// ---------------------------------------------------------------------------
#define PIN_I2C_SDA         21
#define PIN_I2C_SCL         22
#define PIN_LED             2                  // LED azul de la placa: encendido = conectado al WiFi
#define LED_ON              1
#define LED_OFF             0
#define PIN_BUTTON          0                  // botón BOOT: al presionarlo, la posición actual de la mano pasa a ser el "cero"

// ---------------------------------------------------------------------------
// Muestreo y filtrado
// ---------------------------------------------------------------------------
#define SAMPLE_HZ           100                // lectura de la IMU
#define SEND_DIV            2                  // envía 1 de cada 2 muestras -> 50 paquetes/s
#define COMP_ALPHA          0.98f              // filtro complementario: peso del giroscopio frente al acelerómetro
#define IIR_ALPHA           0.25f              // IIR de primer orden sobre los ángulos (suaviza el temblor de la mano)
#define GYRO_CAL_SAMPLES    200                // 2 s con la mano quieta al arrancar
#define NEUTRAL_SAMPLES     100                // 1 s para tomar la posición neutra de la mano
#define LOG_HZ              5                  // líneas por segundo en el monitor (ángulos y temperaturas)

// ---------------------------------------------------------------------------
// MPU-6050
// ---------------------------------------------------------------------------
#define MPU_ADDR            0x68               // AD0 a GND
#define MPU_DLPF_CFG        3                  // filtro interno 44 Hz
