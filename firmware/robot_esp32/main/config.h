// Configuración general del robot minisumo (ESP32 DevKit V1, solo la fila de pines derecha)
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
// Todos en la fila derecha de la DevKit. Libres a propósito:
//   GPIO1 y GPIO3: UART0, la usan la carga del programa y el monitor
//   GPIO2: LED azul de la placa (indica el WiFi); debe estar en bajo al arrancar para poder programar
// GPIO5 y GPIO15 son pines de arranque inofensivos: al encender pueden dar pulsos, pero el motor
// no gira porque AIN/BIN arrancan en bajo.
#define PIN_I2C_SDA         21                 // I2C estándar de la DevKit
#define PIN_I2C_SCL         22

#define PIN_PWMA            5                  // motor izquierdo (A)
#define PIN_AIN1            18
#define PIN_AIN2            19
#define PIN_PWMB            23                 // motor derecho (B)
#define PIN_BIN1            17
#define PIN_BIN2            16

#define PIN_ENC_L           4                  // sensor de herradura izquierdo
#define PIN_ENC_R           15                 // sensor de herradura derecho
#define PIN_LED             2                  // LED de la placa: encendido = WiFi conectado

// ---------------------------------------------------------------------------
// Lazo de control y telemetría
// ---------------------------------------------------------------------------
#define CONTROL_HZ          200                // frecuencia de muestreo de sensores (guía: 200 muestras/s)
#define CONTROL_DT          (1.0f / CONTROL_HZ)
#define TELEM_BATCH         6                  // muestras por paquete UDP -> ~33 paquetes/s (menos trabajo de la radio, menos calor)
#define CMD_TIMEOUT_MS      700                // sin comandos en este tiempo -> motores detenidos (tolera cortes cortos del WiFi)

// ---------------------------------------------------------------------------
// Motores (TB6612FNG + motorreductores TT)
// ---------------------------------------------------------------------------
#define PWM_FREQ_HZ         20000
#define PWM_RES_BITS        10
#define PWM_MAX             0.55f              // límite de ciclo útil: motores de 6 V con batería 3S (12,6 V) -> ~6,5 V
#define MOTOR_L_INVERT      0                  // poner 1 si el motor gira al revés
#define MOTOR_R_INVERT      0
// Aislamiento con optoacopladores: el 6N137 invierte la señal (LED encendido = salida en bajo),
// así que el PWM se genera invertido para que al TB6612 le llegue el ciclo útil correcto.
// Poner 0 si el ESP se conecta directo al TB6612, sin optoacopladores.
#define PWM_OUTPUT_INVERT   0
#define MOTOR_SLEW_PER_S    3.0f               // arranque suave: el ciclo útil cambia como máximo 3/s (0 -> 0,3 en 100 ms)
#define REF_ACCEL_MAX       25.0f              // rampa de la velocidad pedida en rad/s² (0 -> 5 rad/s en 0,2 s): las dos ruedas
                                               // arrancan juntas y el robot no se tuerce al salir

// ---------------------------------------------------------------------------
// Encoders de herradura (un solo canal, sin sentido de giro)
// ---------------------------------------------------------------------------
#define ENC_SLOTS           20                 // ranuras del disco
#define ENC_BOTH_EDGES      0                  // 0: solo subida (20 pulsos/vuelta). Con 1 la ranura y el diente del disco
                                               // no miden igual: los intervalos se alternan y la velocidad salta
#define ENC_DEBOUNCE_US     4000               // ignora flancos más cercanos que esto (permite hasta ~39 rad/s reales)
#define ENC_OMEGA_PHYS_MAX  40.0f              // rad/s: una lectura mayor es físicamente imposible -> ruido
#define ENC_CONFIRM_US      50                 // el pin debe seguir en alto 50 us después del flanco (descarta ruido de los motores)
#define ENC_GLITCH_PCT      70                 // rechaza flancos que llegan antes del 70 % del intervalo anterior: el LM393
                                               // tiembla en el borde de la ranura y con ruido daba un pulso extra por ranura
#define ENC_GLITCH_MAX_US   80000              // tope del umbral adaptativo (a baja velocidad hay ~60 ms entre ranuras)
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
#define PID_KP              0.000f             // apagado con 3S: los encoders aún cuentan ruido de los motores y el PID
                                               // de rueda producía el zigzag. La velocidad la da la prealimentación
#define PID_KI              0.000f             // y el rumbo lo mantiene el giroscopio. Subir con cuidado si se filtran los D0
#define PID_KD              0.000f
#define PID_KFF             0.010f             // prealimentación: ciclo útil por rad/s por encima de la zona muerta
// Zona muerta de cada motor: los motores TT no son iguales (el derecho gira más con el mismo PWM).
// La prealimentación de cada rueda es  kff*ref + duty_min  y el PID solo corrige ±PID_TRIM alrededor
// de ella, así un encoder ruidoso no puede frenar ni disparar la rueda. Ajustables en vivo con F.
#define PID_DUTY_MIN_L      0.10f              // motor izquierdo: necesita más PWM para arrancar
#define PID_DUTY_MIN_R      0.09f              // motor derecho: arranca con menos PWM
// Control de rumbo con el giroscopio: compara el giro real (gz) con el pedido por las ruedas y
// reparte una corrección entre las velocidades pedidas a las ruedas (control en cascada). Hace que
// vaya recto aunque los motores no sean iguales. Ajustable en vivo con Y,kp,ki (Y,0,0 lo desactiva).
// Unidades: rad/s de rueda por rad/s de giro (afinados con 2S; ver pruebas con 3S).
#define YAW_KP              1.0f
#define YAW_KI              17.5f
#define YAW_CORR_MAX        7.5f               // rad/s de rueda (equivale a 0,15 de ciclo útil)

// corrección máxima de ciclo útil
#define PID_TRIM            0.15f              // corrección máxima del PID sobre la prealimentación

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
