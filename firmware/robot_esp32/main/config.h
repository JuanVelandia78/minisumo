// Configuración general del robot minisumo (ESP32 DevKit V1 sobre la PCB_MiniSumo v11)
#pragma once

// ---------------------------------------------------------------------------
// WiFi y red
// ---------------------------------------------------------------------------
#include "wifi_secrets.h"           // WIFI_SSID y WIFI_PASS (no se sube al repositorio, ver wifi_secrets.example.h)
#define WIFI_MAX_TX_POWER   60                 // 0,25 dBm por unidad -> 15 dBm (la antena de la DevKit aguanta más; 78 = 19,5 dBm máx.)

#define UDP_CMD_PORT        8888               // el robot escucha comandos aquí
#define UDP_BEACON_PORT     8889               // el robot anuncia su IP por broadcast aquí (1 Hz)

// ---------------------------------------------------------------------------
// Pines (ver diagrama de cableado)
// ---------------------------------------------------------------------------
// Pines según la PCB_MiniSumo v11 (una cara, DRV8833 rojo, INA219, MPU-6050, encoders de cuadratura)
//   GPIO1 y GPIO3: UART0 (carga del programa y monitor). GPIO2: LED azul de la placa (WiFi).
#define PIN_I2C_SDA         17                 // bus I2C compartido: MPU-6050 (0x68) e INA219 (0x40)
#define PIN_I2C_SCL         16

// DRV8833: no tiene pin de PWM aparte, el PWM se aplica en los pines IN (resistencias de 220 ohm en serie)
#define PIN_AIN1            33                 // motor izquierdo (salidas AO1/AO2)
#define PIN_AIN2            32
#define PIN_BIN1            14                 // motor derecho (salidas BO1/BO2)
#define PIN_BIN2            13
#define PIN_STBY            26                 // nSLEEP del DRV8833: en alto = puente H activo

#define PIN_ENC_L_C1        18                 // encoder izquierdo, canal C1 (A)
#define PIN_ENC_L_C2        19                 // encoder izquierdo, canal C2 (B)
#define PIN_ENC_R_C1        21                 // encoder derecho, canal C1 (A)
#define PIN_ENC_R_C2        22                 // encoder derecho, canal C2 (B)
#define PIN_LED             2                  // LED de la placa: encendido = WiFi conectado

// ---------------------------------------------------------------------------
// Lazo de control y telemetría
// ---------------------------------------------------------------------------
#define CONTROL_HZ          200                // frecuencia de muestreo de sensores (guía: 200 muestras/s)
#define CONTROL_DT          (1.0f / CONTROL_HZ)
#define TELEM_BATCH         6                  // muestras por paquete UDP -> ~33 paquetes/s (menos trabajo de la radio, menos calor)
#define CMD_TIMEOUT_MS      700                // sin comandos en este tiempo -> motores detenidos (tolera cortes cortos del WiFi)

// ---------------------------------------------------------------------------
// Motores (DRV8833 + motorreductores con encoder, batería 2S a través del INA219)
// ---------------------------------------------------------------------------
#define PWM_FREQ_HZ         20000
#define PWM_RES_BITS        10
#define PWM_MAX             0.70f              // límite de ciclo útil: motores de 6 V con batería 2S (8,4 V) -> ~5,9 V
#define MOTOR_L_INVERT      1                  // poner 1 si el motor gira al revés
#define MOTOR_R_INVERT      0
#define MOTOR_SLOW_DECAY    1                  // 1: en el tiempo apagado del PWM el motor frena (como el TB6612,
                                               // velocidad más lineal con el ciclo útil). 0: queda libre (fast decay)
#define MOTOR_SLEW_PER_S    3.0f               // arranque suave: el ciclo útil cambia como máximo 3/s (0 -> 0,3 en 100 ms)
#define REF_ACCEL_MAX       25.0f              // rampa de la velocidad pedida en rad/s² (0 -> 5 rad/s en 0,2 s): las dos ruedas
                                               // arrancan juntas y el robot no se tuerce al salir

// ---------------------------------------------------------------------------
// Encoders de cuadratura del motor (C1/C2), leídos por el contador PCNT en x4
// ---------------------------------------------------------------------------
// Pulsos por vuelta de la RUEDA = ENC_PPR * 4 * ENC_GEAR_RATIO. Verificar: comando R, girar la rueda
// una vuelta a mano y leer ticks_l / ticks_r en la telemetría; ese número es ENC_COUNTS_PER_REV.
#define ENC_PPR             11                 // pulsos por vuelta del eje del motor, por canal (ver hoja del motor)
#define ENC_GEAR_RATIO      34.0f              // reducción de la caja (ver la etiqueta del motor)
#define ENC_COUNTS_PER_REV  2771.0f            // MEDIDO (2026-11-08): 10 vueltas a mano, izq 2779 / der 2763 (~7 PPR x 4 x 100:1)
#define ENC_L_INVERT        1                  // poner 1 si con M,0.3,0.3 los ticks de esa rueda bajan
#define ENC_R_INVERT        0
#define ENC_GLITCH_NS       1000               // filtro del PCNT: ignora pulsos de menos de 1 us (ruido de los motores)

// ---------------------------------------------------------------------------
// Geometría del robot (medir y ajustar)
// ---------------------------------------------------------------------------
#define WHEEL_RADIUS_M      0.0162f            // efectivo: ruedas de ~30 mm, calibrado con recorrido de 33 cm (rueda der.)
#define WHEEL_BASE_M        0.079f             // entre centros de las ruedas (ensamblaje de SolidWorks)

// ---------------------------------------------------------------------------
// PID de velocidad por rueda (ajustables en línea con el comando G)
// ---------------------------------------------------------------------------
#define OMEGA_MAX           13.5f              // rad/s de la rueda con PWM_MAX (medido en el aire con 2S, 2771 cuentas/vuelta)
#define PID_KP              0.020f             // PI de velocidad de rueda (ajustado en el piso con 2S, 2026-11-08)
#define PID_KI              0.400f             // el rumbo lo mantiene el giroscopio (lazo externo)
#define PID_KD              0.000f
#define PID_KFF             0.045f             // prealimentación: ciclo útil por rad/s (medido: ~22 rad/s por unidad de duty)
// Zona muerta de cada motor: los motores TT no son iguales (el derecho gira más con el mismo PWM).
// La prealimentación de cada rueda es  kff*ref + duty_min  y el PID solo corrige ±PID_TRIM alrededor
// de ella, así un encoder ruidoso no puede frenar ni disparar la rueda. Ajustables en vivo con F.
#define PID_DUTY_MIN_L      0.052f             // zona muerta motor izquierdo (curva medida 2026-11-08)
#define PID_DUTY_MIN_R      0.041f             // zona muerta motor derecho
// Control de rumbo con el giroscopio: compara el giro real (gz) con el pedido por las ruedas y
// reparte una corrección entre las velocidades pedidas a las ruedas (control en cascada). Hace que
// vaya recto aunque los motores no sean iguales. Ajustable en vivo con Y,kp,ki (Y,0,0 lo desactiva).
// Unidades: rad/s de rueda por rad/s de giro (afinados con 2S; ver pruebas con 3S).
#define YAW_KP              1.5f               // rad/s de rueda por rad/s de error de giro (amortigua)
#define YAW_KI              40.0f              // rad/s de rueda por rad de error de ángulo (devuelve al rumbo)
#define YAW_KB              20.0f              // integral del ángulo: absorbe el patinaje / motores distintos
#define YAW_ANG_MAX         0.8f               // rad (~45°): error de ángulo máximo que se recuerda
#define YAW_BIAS_MAX        3.0f               // rad/s de rueda: compensación máxima de diferencia entre ruedas
#define YAW_CORR_MAX        6.0f               // rad/s de rueda: corrección total máxima (~0,27 de ciclo útil)

// corrección máxima de ciclo útil
#define PID_TRIM            0.15f              // corrección máxima del PID sobre la prealimentación

// ---------------------------------------------------------------------------
// INA219: voltaje y corriente de la batería 2S (Vin+ = batería, Vin- = VM del DRV8833)
// ---------------------------------------------------------------------------
#define INA_ADDR            0x40               // A0 y A1 a GND
#define INA_SHUNT_OHM       0.1f               // resistencia R100 del módulo
#define INA_READ_DIV        20                 // se lee cada 20 periodos de control -> 10 Hz
#define BAT_LOW_V           6.6f               // 2S: por debajo de 3,3 V por celda se avisa en el monitor
#define BAT_ABSENT_V        3.0f               // por debajo de esto se toma como batería sin conectar (entradas al aire)

// ---------------------------------------------------------------------------
// Autodiagnóstico al arrancar (y con el comando T)
// ---------------------------------------------------------------------------
// El puente H se prueba con un pulso corto y suave a cada motor en cada sentido: con el INA219 se
// mira que circule corriente. Es tan corto que la rueda casi no se mueve (puede oírse un clic).
#define SELFTEST_MOTORS     1                  // 0: no mover los motores en el autodiagnóstico
#define SELFTEST_DUTY       0.15f              // ciclo útil del pulso de prueba
#define SELFTEST_PULSE_MS   40                 // duración del pulso
#define SELFTEST_I_MIN      0.008f             // A: corriente mínima que debe subir para dar el motor por bueno

// ---------------------------------------------------------------------------
// IMU MPU-6050
// ---------------------------------------------------------------------------
#define MPU_ADDR            0x68               // AD0 a GND
#define MPU_DLPF_CFG        2                  // filtro interno: 94 Hz (antialiasing bajo Nyquist = 100 Hz)
#define GYRO_CAL_SAMPLES    400                // 2 s quieto al arrancar
#define GYRO_CAL_MAX_STD    0.5f               // °/s: si el giroscopio varía más, el robot se movió y se repite la calibración
#define GYRO_CAL_TRIES      10
#define GYRO_Z_SIGN         1.0f               // +1: con el montaje de la versión ESP32 el giro antihorario da positivo

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
