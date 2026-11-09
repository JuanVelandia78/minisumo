"""Gráficas en vivo de las señales crudas frente al filtro de Kalman (nodo kalman_filter).

Filas:
  1. IMU del robot: aceleración y velocidad angular (/imu/raw vs Kalman 1D /imu/kalman)
  2. Encoders: velocidad de cada rueda (/wheels/speed vs Kalman [w, alfa] /wheels/kalman)
  3. IMU de la mano: aceleración y velocidad angular (/glove/imu/raw vs /glove/imu/kalman)
  4. Ángulos de la mano: solo acelerómetro vs Kalman giroscopio + acelerómetro (/glove/angles_kalman)

Con -p mostrar_firmware:=true se agrega en azul el filtro que corre en los ESP32
(IIR, media móvil, complementario) para compararlo con el Kalman.

En el título de cada gráfica va la desviación estándar de las dos señales en la ventana:
con el sensor quieto mide el ruido y muestra cuánto lo reduce el filtro.

  ros2 run minisumo_ros kalman_filter        # (ya lo arranca visualizacion.launch.py)
  ros2 run minisumo_ros signal_plotter
  ros2 run minisumo_ros signal_plotter --ros-args -p mostrar_firmware:=true
  ros2 run minisumo_ros signal_plotter --ros-args -p robot_acc:=z -p glove_gyro:=x -p ventana_s:=5.0
"""
import collections
import math
import statistics
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray

try:
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
except ImportError:          # pragma: no cover
    plt = None

G = 9.80665
RAD2DEG = 180.0 / math.pi
AXES = {'x': 0, 'y': 1, 'z': 2}


class SignalPlotter(Node):
    def __init__(self):
        super().__init__('signal_plotter')
        self.declare_parameter('ventana_s', 10.0)     # segundos visibles
        self.declare_parameter('periodo_ms', 300)     # cada cuánto se redibuja (más alto = menos carga)
        self.declare_parameter('max_puntos', 300)     # puntos dibujados por línea (se submuestrea)
        self.declare_parameter('robot_acc', 'x')      # eje de la aceleración del robot
        self.declare_parameter('robot_gyro', 'z')     # eje del giroscopio del robot
        self.declare_parameter('glove_acc', 'x')
        self.declare_parameter('glove_gyro', 'y')
        self.declare_parameter('mostrar_firmware', False)  # agrega el filtro de los ESP32

        self.t0 = time.monotonic()
        self.win = self.get_parameter('ventana_s').value
        self.lock = threading.Lock()
        self.data = collections.defaultdict(collections.deque)

        ax = lambda n: AXES[self.get_parameter(n).value]
        self.ra, self.rg = ax('robot_acc'), ax('robot_gyro')
        self.ga, self.gg = ax('glove_acc'), ax('glove_gyro')

        self.create_subscription(Imu, 'imu/raw', lambda m: self.on_imu(m, 'r', 'raw'), 200)
        self.create_subscription(Imu, 'imu/filtered', lambda m: self.on_imu(m, 'r', 'f'), 200)
        self.create_subscription(Imu, 'glove/imu/raw', lambda m: self.on_imu(m, 'g', 'raw'), 100)
        self.create_subscription(Imu, 'glove/imu/filtered', lambda m: self.on_imu(m, 'g', 'f'), 100)
        self.create_subscription(Float32MultiArray, 'wheels/speed', self.on_wheels, 200)
        self.create_subscription(Float32MultiArray, 'glove/angles', self.on_angles, 100)
        self.create_subscription(Imu, 'imu/kalman', lambda m: self.on_imu(m, 'r', 'k'), 200)
        self.create_subscription(Imu, 'glove/imu/kalman', lambda m: self.on_imu(m, 'g', 'k'), 100)
        self.create_subscription(Float32MultiArray, 'wheels/kalman', self.on_wheels_k, 200)
        self.create_subscription(Float32MultiArray, 'glove/angles_kalman', self.on_angles_k, 100)

    def add(self, key, value):
        t = time.monotonic() - self.t0
        win = self.win
        with self.lock:
            d = self.data[key]
            d.append((t, value))
            while d and d[0][0] < t - win:
                d.popleft()

    def on_imu(self, m, who, kind):
        acc = (m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z)
        gyr = (m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z)
        ia, ig = (self.ra, self.rg) if who == 'r' else (self.ga, self.gg)
        self.add(f'{who}_acc_{kind}', acc[ia])
        self.add(f'{who}_gyr_{kind}', gyr[ig] * RAD2DEG)

    def on_wheels(self, m):
        if len(m.data) >= 4:      # [wl, wr, wl_f, wr_f, ...]
            self.add('wl_raw', m.data[0])
            self.add('wr_raw', m.data[1])
            self.add('wl_f', m.data[2])
            self.add('wr_f', m.data[3])

    def on_wheels_k(self, m):
        if len(m.data) >= 2:
            self.add('wl_k', m.data[0])
            self.add('wr_k', m.data[1])

    def on_angles_k(self, m):
        if len(m.data) >= 2:
            self.add('pitch_k', m.data[0])
            self.add('roll_k', m.data[1])

    def on_angles(self, m):
        if len(m.data) >= 4:      # [pitch, roll, pitch_raw, roll_raw]
            self.add('pitch_f', m.data[0])
            self.add('roll_f', m.data[1])
            self.add('pitch_raw', m.data[2])
            self.add('roll_raw', m.data[3])

    def series(self, key):
        with self.lock:
            d = list(self.data[key])
        vals = [p[1] for p in d]
        # Se dibuja una de cada k muestras para no saturar la VM (la σ usa todas)
        k = max(1, len(d) // self.get_parameter('max_puntos').value)
        return [p[0] for p in d[::k]], vals[::k], vals


def main():
    rclpy.init()
    node = SignalPlotter()
    if plt is None:
        node.get_logger().error('Falta matplotlib: sudo apt install python3-matplotlib')
        return
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    p = lambda n: node.get_parameter(n).value
    # (título, unidad, crudo, Kalman, filtro del ESP32, nombre del filtro del ESP32)
    panels = [
        (f"IMU robot: aceleración {p('robot_acc')}", 'm/s²', 'r_acc_raw', 'r_acc_k', 'r_acc_f', 'IIR'),
        (f"IMU robot: giro {p('robot_gyro')}", '°/s', 'r_gyr_raw', 'r_gyr_k', 'r_gyr_f', 'IIR'),
        ('Encoder izquierdo', 'rad/s', 'wl_raw', 'wl_k', 'wl_f', 'media móvil'),
        ('Encoder derecho', 'rad/s', 'wr_raw', 'wr_k', 'wr_f', 'media móvil'),
        (f"IMU mano: aceleración {p('glove_acc')}", 'm/s²', 'g_acc_raw', 'g_acc_k', 'g_acc_f', 'IIR'),
        (f"IMU mano: giro {p('glove_gyro')}", '°/s', 'g_gyr_raw', 'g_gyr_k', 'g_gyr_f', 'IIR'),
        ('Mano: cabeceo', '°', 'pitch_raw', 'pitch_k', 'pitch_f', 'complementario + IIR'),
        ('Mano: alabeo', '°', 'roll_raw', 'roll_k', 'roll_f', 'complementario + IIR'),
    ]
    fw = p('mostrar_firmware')
    fig, axes = plt.subplots(4, 2, figsize=(13, 9), sharex=True)
    fig.canvas.manager.set_window_title('Minisumo: señales crudas vs filtro de Kalman')
    lines = []
    for ax, (title, unit, kr, kk, kf, fname) in zip(axes.flat, panels):
        lr, = ax.plot([], [], color='0.6', lw=0.8, label='crudo')
        lb = None
        if fw:
            lb, = ax.plot([], [], color='tab:blue', lw=1.0, alpha=0.8, label=f'ESP32 ({fname})')
        lf, = ax.plot([], [], color='tab:red', lw=1.4, label='Kalman')
        ax.set_ylabel(unit)
        ax.grid(alpha=0.3)
        ax.legend(loc='upper left', fontsize=7)
        lines.append((ax, title, kr, kk, kf, lr, lf, lb))
    for ax in axes[-1]:
        ax.set_xlabel('tiempo (s)')
    fig.tight_layout()

    def sd(v):
        return statistics.pstdev(v) if len(v) > 1 else float('nan')

    def update(_):
        now = time.monotonic() - node.t0
        win = node.get_parameter('ventana_s').value
        for ax, title, kr, kk, kf, lr, lf, lb in lines:
            tr, vr, allr = node.series(kr)
            tk, vk, allk = node.series(kk)
            lr.set_data(tr, vr)
            lf.set_data(tk, vk)
            vals = vr + vk
            extra = ''
            if lb is not None:
                tb, vb, allb = node.series(kf)
                lb.set_data(tb, vb)
                vals += vb
                extra = f' | σ ESP32 {sd(allb):.3g}'
            ax.set_xlim(max(0.0, now - win), max(win, now))
            if vals:
                lo, hi = min(vals), max(vals)
                pad = 0.1 * (hi - lo) + 1e-3
                ax.set_ylim(lo - pad, hi + pad)
                ax.set_title(f'{title}   σ crudo {sd(allr):.3g} | σ Kalman {sd(allk):.3g}{extra}', fontsize=9)
            else:
                ax.set_title(f'{title}   (sin datos)', fontsize=9)
        return []

    anim = FuncAnimation(fig, update, interval=p('periodo_ms'), cache_frame_data=False)  # noqa: F841
    plt.show()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
