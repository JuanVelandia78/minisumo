"""Gráficas en vivo de las señales antes y después del filtrado.

Filas:
  1. IMU del robot: aceleración y velocidad angular (crudo /imu/raw vs filtrado IIR /imu/filtered)
  2. Encoders: velocidad de cada rueda (crudo vs media móvil, /wheels/speed)
  3. IMU de la mano: aceleración y velocidad angular (/glove/imu/raw vs /glove/imu/filtered)
  4. Ángulos de la mano: solo acelerómetro (crudo) vs complementario + IIR (/glove/angles)

En el título de cada gráfica va la desviación estándar de las dos señales en la ventana:
con el sensor quieto mide el ruido y muestra cuánto lo reduce el filtro.

  ros2 run minisumo_ros signal_plotter
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
    panels = [
        (f"IMU robot: aceleración {p('robot_acc')}", 'm/s²', 'r_acc_raw', 'r_acc_f', 'IIR'),
        (f"IMU robot: giro {p('robot_gyro')}", '°/s', 'r_gyr_raw', 'r_gyr_f', 'IIR'),
        ('Encoder izquierdo', 'rad/s', 'wl_raw', 'wl_f', 'media móvil'),
        ('Encoder derecho', 'rad/s', 'wr_raw', 'wr_f', 'media móvil'),
        (f"IMU mano: aceleración {p('glove_acc')}", 'm/s²', 'g_acc_raw', 'g_acc_f', 'IIR'),
        (f"IMU mano: giro {p('glove_gyro')}", '°/s', 'g_gyr_raw', 'g_gyr_f', 'IIR'),
        ('Mano: cabeceo', '°', 'pitch_raw', 'pitch_f', 'complementario + IIR'),
        ('Mano: alabeo', '°', 'roll_raw', 'roll_f', 'complementario + IIR'),
    ]
    fig, axes = plt.subplots(4, 2, figsize=(13, 9), sharex=True)
    fig.canvas.manager.set_window_title('Minisumo: señales antes y después del filtrado')
    lines = []
    for ax, (title, unit, kr, kf, fname) in zip(axes.flat, panels):
        lr, = ax.plot([], [], color='0.6', lw=0.8, label='crudo')
        lf, = ax.plot([], [], color='tab:red', lw=1.4, label=f'filtrado ({fname})')
        ax.set_ylabel(unit)
        ax.grid(alpha=0.3)
        ax.legend(loc='upper left', fontsize=7)
        lines.append((ax, title, kr, kf, lr, lf))
    for ax in axes[-1]:
        ax.set_xlabel('tiempo (s)')
    fig.tight_layout()

    def sd(v):
        return statistics.pstdev(v) if len(v) > 1 else float('nan')

    def update(_):
        now = time.monotonic() - node.t0
        win = node.get_parameter('ventana_s').value
        for ax, title, kr, kf, lr, lf in lines:
            tr, vr, allr = node.series(kr)
            tf, vf, allf = node.series(kf)
            lr.set_data(tr, vr)
            lf.set_data(tf, vf)
            ax.set_xlim(max(0.0, now - win), max(win, now))
            vals = vr + vf
            if vals:
                lo, hi = min(vals), max(vals)
                pad = 0.1 * (hi - lo) + 1e-3
                ax.set_ylim(lo - pad, hi + pad)
                ax.set_title(f'{title}   σ crudo {sd(allr):.3g} | σ filtrado {sd(allf):.3g}', fontsize=9)
            else:
                ax.set_title(f'{title}   (sin datos)', fontsize=9)
        return []

    anim = FuncAnimation(fig, update, interval=p('periodo_ms'), cache_frame_data=False)  # noqa: F841
    plt.show()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
