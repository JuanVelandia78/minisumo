"""Genera las figuras del informe y del manual en docs/img/.

Fuentes:
  - Coeficientes de los filtros: firmware (filter_coefs.h, config.h del robot y del guante)
  - Filtros de Kalman: clases de ros2/minisumo_ros/minisumo_ros/kalman_filter.py (se importan tal cual)
  - Curva de los motores y recuperación de rumbo: datos medidos en las pruebas (tablas abajo)
  - Robot 3D: mallas de ros2/minisumo_ros/meshes (convertidas del ensamblaje de SolidWorks)

  python herramientas/figuras_informe.py
"""
import math
import os
import sys
import types

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from scipy import signal

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
OUT = os.path.join(ROOT, 'docs', 'img')
os.makedirs(OUT, exist_ok=True)

# Paleta del proyecto (DESIGN.md)
INK, MUTED, RULE, CAR, FUSA, UMBER, RED, AMBER = '#1d1a17', '#6f655c', '#d9d2c7', '#d9a400', '#2f9e7f', '#b0764f', '#c8322f', '#d98a1c'
plt.rcParams.update({
    'font.family': 'DejaVu Sans', 'font.size': 10, 'axes.edgecolor': '#9a9088', 'axes.labelcolor': INK,
    'axes.titleweight': 'bold', 'axes.titlesize': 11, 'axes.grid': True, 'grid.color': '#e6e0d6', 'grid.linewidth': .8,
    'xtick.color': MUTED, 'ytick.color': MUTED, 'axes.spines.top': False, 'axes.spines.right': False,
    'legend.frameon': False, 'figure.dpi': 110, 'savefig.dpi': 160, 'savefig.bbox': 'tight'})


def save(fig, name):
    fig.savefig(os.path.join(OUT, name))
    plt.close(fig)
    print('  ', name)


# ---------------------------------------------------------------- 1. IIR Butterworth
b = np.array([0.06745527, 0.13491055, 0.06745527])
a = np.array([1.0, -1.14298050, 0.41280160])
fig, ax = plt.subplots(1, 2, figsize=(10, 3.6))
for fs, lab, c in ((200, 'Robot (fs = 200 Hz, fc = 20 Hz)', INK), (100, 'Guante (fs = 100 Hz, fc = 10 Hz)', UMBER)):
    w, h = signal.freqz(b, a, worN=4096, fs=fs)
    ax[0].plot(w, 20 * np.log10(np.maximum(abs(h), 1e-6)), color=c, lw=2, label=lab)
    _, gd = signal.group_delay((b, a), w=4096, fs=fs)
    ax[1].plot(w, gd / fs * 1000, color=c, lw=2, label=lab)
ax[0].axhline(-3, color=RED, lw=1, ls='--'); ax[0].text(70, -1.5, '−3 dB', color=RED)
ax[0].set(xlim=(0, 100), ylim=(-60, 3), xlabel='Frecuencia (Hz)', ylabel='Magnitud (dB)', title='IIR Butterworth orden 2: magnitud')
ax[0].legend(loc='lower left')
ax[1].set(xlim=(0, 50), xlabel='Frecuencia (Hz)', ylabel='Retardo de grupo (ms)', title='Retardo de grupo')
save(fig, 'iir_respuesta.png')

# ---------------------------------------------------------------- 2. Media móvil vs IIR
fs = 200
fig, ax = plt.subplots(figsize=(6.4, 3.4))
for N, c in ((5, FUSA),):
    w, h = signal.freqz(np.ones(N) / N, [1], worN=4096, fs=fs)
    ax.plot(w, 20 * np.log10(np.maximum(abs(h), 1e-6)), color=c, lw=2, label=f'Media móvil N = {N} (ruedas)')
w, h = signal.freqz(b, a, worN=4096, fs=fs)
ax.plot(w, 20 * np.log10(np.maximum(abs(h), 1e-6)), color=INK, lw=1.4, ls='--', label='IIR Butterworth (IMU)')
for k in (40, 80):
    ax.axvline(k, color=MUTED, lw=.8, ls=':')
ax.text(41, -55, 'ceros en fs/N = 40 Hz y 80 Hz', color=MUTED, fontsize=8.5)
ax.set(xlim=(0, 100), ylim=(-60, 3), xlabel='Frecuencia (Hz)', ylabel='Magnitud (dB)', title='Media móvil de 5 muestras frente al IIR (fs = 200 Hz)')
ax.legend(loc='lower left')
save(fig, 'media_movil.png')

# ---------------------------------------------------------------- 3. Complementario del guante
alpha, dt = 0.98, 0.01
f = np.logspace(-2, np.log10(50), 600)
z = np.exp(1j * 2 * np.pi * f * dt)
H_acc = (1 - alpha) / (1 - alpha * z ** -1)            # pasa-bajos sobre el acelerómetro
H_gyr = alpha * (1 - z ** -1) / (1 - alpha * z ** -1)   # giro integrado: pasa-altos sobre el ángulo
fc = (1 - alpha) / (2 * np.pi * alpha * dt)
fig, ax = plt.subplots(figsize=(6.4, 3.4))
ax.semilogx(f, 20 * np.log10(abs(H_acc)), color=UMBER, lw=2, label='Acelerómetro → pasa-bajos')
ax.semilogx(f, 20 * np.log10(abs(H_gyr)), color=FUSA, lw=2, label='Giroscopio integrado → pasa-altos')
ax.semilogx(f, 20 * np.log10(abs(H_acc + H_gyr)), color=INK, lw=1.2, ls='--', label='Suma = 1 (0 dB)')
ax.axvline(fc, color=RED, lw=1, ls='--'); ax.text(fc * 1.1, -32, f'fc ≈ {fc:.2f} Hz', color=RED)
ax.set(ylim=(-40, 5), xlabel='Frecuencia (Hz)', ylabel='Magnitud (dB)', title=f'Filtro complementario del guante (α = {alpha}, Δt = {dt * 1000:.0f} ms)')
ax.legend(loc='lower left', fontsize=8.5)
save(fig, 'complementario.png')

# ---------------------------------------------------------------- 4. Kalman (clases reales del paquete)
for m in ['rclpy', 'rclpy.node', 'sensor_msgs', 'sensor_msgs.msg', 'std_msgs', 'std_msgs.msg']:
    sys.modules[m] = types.ModuleType(m)
sys.modules['rclpy.node'].Node = object
sys.modules['sensor_msgs.msg'].Imu = object
sys.modules['std_msgs.msg'].Float32MultiArray = object
sys.path.insert(0, os.path.join(ROOT, 'ros2', 'minisumo_ros', 'minisumo_ros'))
from kalman_filter import Kalman1D, KalmanVel, KalmanAngle  # noqa: E402

rng = np.random.default_rng(7)
fig, ax = plt.subplots(3, 1, figsize=(9, 8.4))
# a) acelerómetro: escalón (choque) con ruido
t = np.arange(0, 4, 0.005)
true = np.where((t > 1.5) & (t < 2.4), 2.5, 0.0)
raw = true + rng.normal(0, 0.5, t.size)
k = Kalman1D(0.005, 0.25); kal = np.array([k.update(z) for z in raw])
zi = signal.lfilter_zi(b, a) * 0; iir, _ = signal.lfilter(b, a, raw, zi=zi)
ax[0].plot(t, raw, color='#b9b1a6', lw=.7, label='crudo')
ax[0].plot(t, iir, color=UMBER, lw=1.4, label='IIR del ESP32')
ax[0].plot(t, kal, color=FUSA, lw=2, label='Kalman 1D (PC)')
ax[0].plot(t, true, color=INK, lw=1, ls='--', label='real')
ax[0].set(title='Acelerómetro: Kalman 1D (Q = 0,005, R = 0,25)', ylabel='m/s²'); ax[0].legend(ncol=4, fontsize=8.5, loc='upper left')
# b) rueda: rampa a 6 rad/s con ruido de cuantización
dtw = 0.005; t2 = np.arange(0, 3, dtw)
w_true = np.clip((t2 - 0.8) * 25, 0, 6)
w_raw = w_true + rng.normal(0, 0.4, t2.size)
kv = KalmanVel(50, 0.16); w_k = np.array([kv.update(z, dtw) for z in w_raw])
w_ma = np.convolve(w_raw, np.ones(5) / 5)[:t2.size]
ax[1].plot(t2, w_raw, color='#b9b1a6', lw=.7, label='crudo')
ax[1].plot(t2, w_ma, color=UMBER, lw=1.4, label='media móvil del ESP32')
ax[1].plot(t2, w_k, color=FUSA, lw=2, label='Kalman [ω, α] (PC)')
ax[1].plot(t2, w_true, color=INK, lw=1, ls='--', label='real')
ax[1].set(title='Velocidad de rueda: Kalman de 2 estados (velocidad constante + aceleración aleatoria)', ylabel='rad/s'); ax[1].legend(ncol=4, fontsize=8.5, loc='upper left')
# c) ángulo de la mano: giro con sesgo + acelerómetro ruidoso
dta = 0.01; t3 = np.arange(0, 20, dta)
th = 30 * np.sin(0.5 * t3); rate = 15 * np.cos(0.5 * t3)
g = rate + 2.0 + rng.normal(0, 0.5, t3.size); acc = th + rng.normal(0, 2.0, t3.size)
ka = KalmanAngle(0.01, 0.003, 0.5); est = []; bias = []
for zz, gg in zip(acc, g):
    est.append(ka.update(zz, gg, dta)); bias.append(ka.b)
gyro_only = np.cumsum(g) * dta
ax[2].plot(t3, acc, color='#b9b1a6', lw=.7, label='solo acelerómetro')
ax[2].plot(t3, gyro_only, color=AMBER, lw=1.2, label='solo giroscopio (deriva)')
ax[2].plot(t3, est, color=FUSA, lw=2, label='Kalman [θ, sesgo]')
ax[2].plot(t3, th, color=INK, lw=1, ls='--', label='real')
ax[2].set(title='Ángulo de la mano: fusión giroscopio + acelerómetro con estimación del sesgo', xlabel='Tiempo (s)', ylabel='grados', ylim=(-45, 70))
ax[2].legend(ncol=4, fontsize=8.5, loc='upper left')
fig.tight_layout()
save(fig, 'kalman_demo.png')
e_acc = np.sqrt(np.mean((acc[1000:] - th[1000:]) ** 2)); e_k = np.sqrt(np.mean((np.array(est[1000:]) - th[1000:]) ** 2))
print(f'   ángulo: RMS acelerómetro {e_acc:.2f}°, Kalman {e_k:.2f}°, sesgo estimado {bias[-1]:.2f} °/s')
print(f'   acelerómetro: σ crudo {raw[:250].std():.3f}, Kalman {kal[50:250].std():.3f}, IIR {iir[50:250].std():.3f}')
print(f'   rueda: σ crudo {(w_raw - w_true)[300:].std():.3f}, Kalman {(w_k - w_true)[300:].std():.3f}, media móvil {(w_ma - w_true)[300:].std():.3f}')

# ---------------------------------------------------------------- 5. Curva de los motores (medida)
duty = np.array([0.10, 0.15, 0.20, 0.30, 0.40, 0.50, 0.60, 0.70])
k_cpr = 1496 / 2771      # el barrido se midió con 1496 cuentas/vuelta; las reales son 2771
wl = np.array([1.87, 3.56, 5.48, 9.31, 13.75, 17.30, 21.13, 25.50]) * k_cpr
wr = np.array([2.46, 4.48, 6.68, 10.95, 13.54, 18.94, 23.28, 27.51]) * k_cpr
fig, ax = plt.subplots(figsize=(6.4, 3.6))
ax.plot(duty, wl, 'o-', color=INK, lw=2, label='Rueda izquierda')
ax.plot(duty, wr, 's-', color=CAR, lw=2, label='Rueda derecha')
p = np.polyfit(duty, (wl + wr) / 2, 1)
ax.plot(duty, np.polyval(p, duty), color=MUTED, lw=1, ls='--', label=f'ajuste: ω ≈ {p[0]:.1f}·d {p[1]:+.1f}')
ax.set(xlabel='Ciclo útil del PWM', ylabel='Velocidad de rueda (rad/s)', title='Curva medida de los motores (lazo abierto, ruedas al aire, 2S a 8 V)')
ax.legend()
save(fig, 'curva_motores.png')

# ---------------------------------------------------------------- 6. Recuperación de rumbo (medida)
t_new = [0.00, 0.20, 0.40, 0.60, 0.80, 1.00, 1.20, 1.40, 1.60, 1.80, 2.00, 2.20, 2.40, 2.60, 2.81, 3.00, 3.21, 3.40, 3.60, 3.81, 4.00, 4.21, 4.41, 4.61, 4.80, 5.00, 5.21, 5.41, 5.61, 5.80]
r_new = [0.0, 0.0, 0.4, 0.4, 0.4, 0.0, 0.1, -0.1, 0.2, 0.3, 0.5, 0.2, -0.4, 0.8, 0.4, 0.1, -0.2, 1.3, 6.3, 16.5, 24.5, 6.2, -2.3, -1.9, -3.3, -2.9, -3.3, -2.6, -2.5, -1.7]
t_old = [0.00, 0.20, 0.40, 0.60, 0.80, 1.00, 1.20, 1.40, 1.60, 1.80, 2.00, 2.20, 2.40, 2.60, 2.81, 3.00, 3.21, 3.40, 3.60, 3.81, 4.00, 4.21, 4.41, 4.61, 4.80, 5.00, 5.21, 5.40, 5.61, 5.80]
r_old = [0.0, 0.5, 1.5, 1.3, 1.1, 1.2, 0.8, 0.7, 0.8, 1.1, 1.1, 1.4, 1.4, 1.6, 1.2, 1.4, 1.6, 1.8, 1.3, 3.4, 7.6, 8.3, 7.3, 7.4, 7.6, 7.1, 7.3, 6.8, 6.2, 6.2]
fig, ax = plt.subplots(figsize=(7.6, 3.6))
ax.axhspan(-3, 3, color=FUSA, alpha=.12, lw=0); ax.text(0.05, 3.6, 'banda ±3°', color=FUSA, fontsize=8.5)
ax.plot(t_old, r_old, 'o-', color=MUTED, lw=1.5, ms=3.5, label='Control anterior (solo velocidad de giro): queda torcido')
ax.plot(t_new, r_new, 'o-', color=INK, lw=2, ms=3.5, label='Control nuevo (mantiene el ángulo): vuelve al rumbo')
ax.annotate('empujón', xy=(4.0, 24.5), xytext=(2.6, 22), color=RED, arrowprops=dict(arrowstyle='->', color=RED))
ax.set(xlabel='Tiempo (s)', ylabel='Rumbo (grados)', title='Perturbación lateral a 6 rad/s: recuperación medida del rumbo (giroscopio)')
ax.legend(loc='upper left', fontsize=8.5)
save(fig, 'recuperacion_rumbo.png')

# ---------------------------------------------------------------- 7. Robot 3D (mallas del CAD)
M = os.path.join(ROOT, 'ros2', 'minisumo_ros', 'meshes')


def load(n):
    bts = open(os.path.join(M, n), 'rb').read()
    k = int.from_bytes(bts[80:84], 'little')
    return np.frombuffer(bts[84:84 + k * 50], dtype=np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')]))['v'] * 1000


parts = [('chasis.stl', (.30, .30, .32), (0, 0, 0)), ('soportes.stl', (.82, .82, .85), (0, 0, 0)), ('motores.stl', (.62, .64, .68), (0, 0, 0)),
         ('rueda_izq.stl', (.95, .76, .07), (0, 39.5, 16.2)), ('rueda_der.stl', (.95, .76, .07), (0, -39.5, 16.2))]
L = np.array([-.4, -.7, .9]); L /= np.linalg.norm(L)
for name, (el, az), bg in (('robot_3d.png', (24, -128), 'none'), ('robot_3d_frente.png', (18, -40), 'none')):
    fig = plt.figure(figsize=(6, 5), facecolor=bg)
    ax = fig.add_subplot(projection='3d', facecolor=bg)
    for n, c, off in parts:
        v = load(n) + np.array(off)
        nr = np.cross(v[:, 1] - v[:, 0], v[:, 2] - v[:, 0]); nr /= np.maximum(np.linalg.norm(nr, axis=1, keepdims=True), 1e-9)
        sh = .35 + .85 * np.abs(nr @ L)
        ax.add_collection3d(Poly3DCollection(v, facecolors=np.clip(np.outer(sh, c), 0, 1), edgecolors='none'))
    ax.set_xlim(-22, 88); ax.set_ylim(-55, 55); ax.set_zlim(-14, 96); ax.set_box_aspect((1, 1, 1)); ax.view_init(el, az); ax.set_axis_off()
    fig.subplots_adjust(0, 0, 1, 1)
    fig.savefig(os.path.join(OUT, name), transparent=True); plt.close(fig); print('  ', name)

print('Figuras en', OUT)
