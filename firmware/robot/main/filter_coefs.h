// Coeficientes del filtro IIR de la IMU.
// Reemplazar por el diseño de MATLAB de las partes anteriores de la práctica.
#pragma once

// IIR Butterworth pasa-bajos de orden 2, fs = 200 Hz, fc = 20 Hz.
// MATLAB: [b, a] = butter(2, 20/(200/2));  -> {b0, b1, b2, a1, a2} con a0 = 1
// Para órdenes mayores: [sos, g] = tf2sos(b, a); una fila por sección
// (multiplicar la ganancia g en los b de la primera sección). Máximo 4 secciones.
static const float IIR_IMU_SOS[][5] = {
    {0.06745527f, 0.13491055f, 0.06745527f, -1.14298050f, 0.41280160f},
};
#define IIR_IMU_SECTIONS (sizeof(IIR_IMU_SOS) / sizeof(IIR_IMU_SOS[0]))
