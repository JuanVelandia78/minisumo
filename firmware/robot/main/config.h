// Configuración general del robot minisumo (ESP32-C3 Super Mini)
#pragma once

// ---------------------------------------------------------------------------
// WiFi y red
// ---------------------------------------------------------------------------
#include "wifi_secrets.h"           // WIFI_SSID y WIFI_PASS (no se sube al repositorio, ver wifi_secrets.example.h)
#define WIFI_MAX_TX_POWER   34                 // 0,25 dBm por unidad -> 8,5 dBm (la Super Mini falla con potencia alta)

#define UDP_CMD_PORT        8888               // el robot escucha comandos aquí
#define UDP_BEACON_PORT     8889               // el robot anuncia su IP por broadcast aquí (1 Hz)

// ---------------------------------------------------------------------------
// Pines (ver diagrama de cableado)
// ---------------------------------------------------------------------------
#define PIN_I2C_SDA         8
#define PIN_I2C_SCL         9

#define PIN_PWMA            5                  // motor izquierdo (A)
#define PIN_AIN1            6
#define PIN_AIN2            7
#define PIN_PWMB            10                 // motor derecho (B)
#define PIN_BIN1            20
#define PIN_BIN2            21

#define PIN_ENC_L           3                  // sensor de herradura izquierdo
#define PIN_ENC_R           4                  // sensor de herradura derecho

// ---------------------------------------------------------------------------
// Lazo de control y telemetría
// ---------------------------------------------------------------------------
#define CONTROL_HZ          200                // frecuencia de muestreo de sensores (guía: 200 muestras/s)
#define CONTROL_DT          (1.0f / CONTROL_HZ)
#define TELEM_BATCH         4                  // muestras por paquete UDP -> 50 paquetes/s
#define CMD_TIMEOUT_MS      300                // sin comandos en este tiempo -> motores detenidos

// ---------------------------------------------------------------------------
// Motores (TB6612FNG + motorreductores TT)
// ---------------------------------------------------------------------------
#define PWM_FREQ_HZ         20000
#define PWM_RES_BITS        10
#define PWM_MAX             0.70f              // límite de ciclo útil: motores de 6 V con batería 2S (8,4 V)
#define MOTOR_L_INVERT      0                  // poner 1 si el motor gira al revés
#define MOTOR_R_INVERT      0
#define MOTOR_SLEW_PER_S    3.0f               // arranque suave: el ciclo útil cambia como máximo 3/s (0 -> 0,3 en 100 ms)

// ---------------------------------------------------------------------------
// Encoders de herradura (un solo canal, sin sentido de giro)
// ---------------------------------------------------------------------------
#define ENC_SLOTS           20                 // ranuras del disco
#define ENC_BOTH_EDGES      1                  // 1: cuenta subida y bajada (40 pulsos/vuelta), 0: solo subida
#define ENC_DEBOUNCE_US     4000               // ignora flancos más cercanos que esto (permite hasta ~39 rad/s reales)
#define ENC_OMEGA_PHYS_MAX  25.0f              // rad/s: una lectura mayor es físicamente imposible -> ruido, se descarta
#define ENC_GLITCH_PCT      35                 // rechaza flancos que llegan antes del 35 % del intervalo anterior (ruido)
#define ENC_GLITCH_MAX_US   10000              // tope del umbral adaptativo (no rechaza aceleraciones reales)
#define ENC_STOP_TIMEOUT_US 150000             // sin flancos en este tiempo -> velocidad 0

// ---------------------------------------------------------------------------
// Geometría del robot (medir y ajustar)
// ---------------------------------------------------------------------------
#define WHEEL_RADIUS_M      0.0325f            // rueda TT de 65 mm
#define WHEEL_BASE_M        0.085f             // distancia entre los centros de las ruedas

// ---------------------------------------------------------------------------
// PID de velocidad por rueda (ajustables en línea con el comando G)
// ---------------------------------------------------------------------------
#define OMEGA_MAX           25.0f              // rad/s aproximados de la rueda con PWM_MAX (ajustado con las mediciones)
#define PID_KP              0.020f             // ciclo útil por rad/s
#define PID_KI              0.250f
#define PID_KD              0.000f
#define PID_KFF             0.020f             // prealimentación: ciclo útil por rad/s por encima de la zona muerta
// Zona muerta de cada motor: los motores TT no son iguales (el derecho gira más con el mismo PWM).
// La prealimentación de cada rueda es  kff*ref + duty_min  y el PID solo corrige ±PID_TRIM alrededor
// de ella, así un encoder ruidoso no puede frenar ni disparar la rueda. Ajustables en vivo con F.
#define PID_DUTY_MIN_L      0.15f              // motor izquierdo: necesita más PWM para arrancar
#define PID_DUTY_MIN_R      0.085f             // motor derecho: arranca con menos PWM
// Control de rumbo con el giroscopio: compara el giro real (gz) con el pedido por las ruedas y
// reparte una corrección entre ambos motores. Hace que vaya recto aunque los motores no sean iguales
// y aunque un encoder tenga ruido. Ajustable en vivo con Y,kp,ki (Y,0,0 lo desactiva).
#define YAW_KP              0.08f
#define YAW_KI              1.00f
#define YAW_CORR_MAX        0.30f   // corrección máxima de ciclo útil
#define PID_TRIM            0.03f              // corrección máxima del PID sobre la prealimentación

// ---------------------------------------------------------------------------
// IMU MPU-6050
// ---------------------------------------------------------------------------
#define MPU_ADDR            0x68               // AD0 a GND
#define MPU_DLPF_CFG        2                  // filtro interno: 94 Hz (antialiasing bajo Nyquist = 100 Hz)
#define GYRO_CAL_SAMPLES    400                // 2 s quieto al arrancar
#define GYRO_Z_SIGN        -1.0f               // -1: la IMU está montada de forma que el giro antihorario da negativo

// ---------------------------------------------------------------------------
// Estimación de posición
// ---------------------------------------------------------------------------
#define GYRO_BIAS_TRACK     0.002f             // con el robot quieto el bias se reajusta solo (~2,5 s)
#define HEADING_ALPHA       0.98f              // peso del giroscopio frente a los encoders en el rumbo

// ---------------------------------------------------------------------------
// Filtros digitales
//   IMU (6 ejes)          -> IIR, coeficientes en filter_coefs.h
//   Velocidad de ruedas   -> media móvil de MOVAVG_N muestras
// ---------------------------------------------------------------------------
#define MOVAVG_N            5                  // 5 muestras = ventana de 25 ms a 200 Hz
