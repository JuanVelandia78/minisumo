// Autodiagnóstico de la PCB: bus I2C, MPU-6050, INA219/batería y puente H (DRV8833 + motores).
// Corre al arrancar y con el comando T. Imprime un informe en el monitor serie.
#pragma once
#include <stdbool.h>

// Devuelve el número de fallas encontradas (0 = todo bien). Usa el bus I2C y los motores:
// llamarlo solo desde la tarea que maneja el bus (o antes de crear las tareas).
int selftest_run(bool imu_ok, bool ina_ok);
// Parpadeo del LED de la placa: 3 lentos = todo bien, 10 rápidos = hay fallas
void selftest_blink(int fails);
