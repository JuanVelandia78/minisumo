"""Filtros de Kalman sobre las señales crudas del robot y del guante (en el PC, con ROS2).

Entradas (crudas)            Filtro                                         Salida
  /imu/raw                   Kalman 1D por eje (paseo aleatorio)            /imu/kalman
  /wheels/speed [wl, wr]     Kalman 2 estados por rueda: [ω, α]             /wheels/kalman [wl, wr, αl, αr]
                             (velocidad constante + aceleración aleatoria)
  /glove/imu/raw             Kalman 1D por eje                              /glove/imu/kalman
  /glove/angles (crudos) +   Kalman 2 estados por ángulo: [θ, sesgo giro]   /glove/angles_kalman
  giro de /glove/imu/raw     predice con el giroscopio, corrige con el      [cabeceo, alabeo, sesgo_c, sesgo_a]
                             ángulo del acelerómetro

Ajuste: Q (ruido del proceso) alto = sigue más rápido pero filtra menos; R (ruido de la medida)
alto = confía menos en el sensor y suaviza más. Con el sensor quieto, R ≈ varianza del ruido.

  ros2 run minisumo_ros kalman_filter
  ros2 run minisumo_ros kalman_filter --ros-args -p rueda_q:=100.0 -p ang_r:=1.0
"""
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray

RAD2DEG = 180.0 / math.pi


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


class Kalman1D:
    """x_k = x_{k-1} + w (Q),  z_k = x_k + v (R)."""

    def __init__(self, q, r):
        self.q, self.r = q, r
        self.x, self.p, self.init = 0.0, 1.0, False

    def update(self, z):
        if not self.init:
            self.x, self.p, self.init = z, self.r, True
            return self.x
        self.p += self.q                        # predicción
        k = self.p / (self.p + self.r)          # ganancia
        self.x += k * (z - self.x)              # corrección
        self.p *= (1.0 - k)
        return self.x


class KalmanVel:
    """Estado [ω, α]: ω_k = ω + α·dt, α_k = α + ruido (aceleración aleatoria de densidad q). Medida: ω."""

    def __init__(self, q, r):
        self.q, self.r = q, r
        self.w = self.a = 0.0
        self.P = [[1.0, 0.0], [0.0, 1.0]]
        self.init = False

    def update(self, z, dt):
        if not self.init:
            self.w, self.init = z, True
            self.P = [[self.r, 0.0], [0.0, 10.0]]
            return self.w
        # Predicción: x = F x,  P = F P Fᵀ + Q,  F = [[1, dt], [0, 1]]
        self.w += self.a * dt
        p00, p01, p10, p11 = self.P[0][0], self.P[0][1], self.P[1][0], self.P[1][1]
        q = self.q
        p00 = p00 + dt * (p10 + p01) + dt * dt * p11 + q * dt ** 3 / 3.0
        p01 = p01 + dt * p11 + q * dt ** 2 / 2.0
        p10 = p10 + dt * p11 + q * dt ** 2 / 2.0
        p11 = p11 + q * dt
        # Corrección con H = [1, 0]
        s = p00 + self.r
        k0, k1 = p00 / s, p10 / s
        y = z - self.w
        self.w += k0 * y
        self.a += k1 * y
        self.P = [[(1 - k0) * p00, (1 - k0) * p01],
                  [p10 - k1 * p00, p11 - k1 * p01]]
        return self.w


class KalmanAngle:
    """Estado [θ, b] (°, °/s): θ_k = θ + (giro - b)·dt. Medida: ángulo del acelerómetro."""

    def __init__(self, q_ang, q_bias, r):
        self.qa, self.qb, self.r = q_ang, q_bias, r
        self.th = self.b = 0.0
        self.P = [[0.0, 0.0], [0.0, 0.0]]
        self.init = False

    def update(self, z, rate, dt):
        if not self.init:
            self.th, self.init = z, True
            return self.th
        # Predicción con el giroscopio
        self.th = wrap180(self.th + (rate - self.b) * dt)
        P = self.P
        p00 = P[0][0] + dt * (dt * P[1][1] - P[0][1] - P[1][0] + self.qa)
        p01 = P[0][1] - dt * P[1][1]
        p10 = P[1][0] - dt * P[1][1]
        p11 = P[1][1] + self.qb * dt
        # Corrección con el acelerómetro (innovación envuelta a ±180°)
        s = p00 + self.r
        k0, k1 = p00 / s, p10 / s
        y = wrap180(z - self.th)
        self.th = wrap180(self.th + k0 * y)
        self.b += k1 * y
        self.P = [[p00 - k0 * p00, p01 - k0 * p01],
                  [p10 - k1 * p00, p11 - k1 * p01]]
        return self.th


class KalmanNode(Node):
    def __init__(self):
        super().__init__('kalman_filter')
        par = {
            'imu_acc_q': 0.005, 'imu_acc_r': 0.25,      # (m/s²)²: IMU del robot y del guante
            'imu_gyro_q': 2e-5, 'imu_gyro_r': 8e-5,     # (rad/s)²
            'rueda_q': 50.0, 'rueda_r': 0.16,           # densidad de aceleración (rad/s²)²·s, medida (rad/s)²
            'rueda_dt': 0.005,                          # s entre muestras de la telemetría (200 Hz)
            'ang_q': 0.01, 'ang_q_bias': 0.003, 'ang_r': 0.5,   # ángulos de la mano (°², (°/s)², °²)
        }
        for k, v in par.items():
            self.declare_parameter(k, v)
        g = lambda n: self.get_parameter(n).value

        def imu_set():
            return ([Kalman1D(g('imu_acc_q'), g('imu_acc_r')) for _ in range(3)] +
                    [Kalman1D(g('imu_gyro_q'), g('imu_gyro_r')) for _ in range(3)])
        self.k_imu_r = imu_set()
        self.k_imu_g = imu_set()
        self.k_wheel = [KalmanVel(g('rueda_q'), g('rueda_r')) for _ in range(2)]
        self.k_ang = [KalmanAngle(g('ang_q'), g('ang_q_bias'), g('ang_r')) for _ in range(2)]
        self.wheel_dt = g('rueda_dt')
        self.glove_raw_ang = None          # (cabeceo, alabeo) solo acelerómetro, respecto al neutro
        self.glove_t = None

        self.pub_imu = self.create_publisher(Imu, 'imu/kalman', 200)
        self.pub_gimu = self.create_publisher(Imu, 'glove/imu/kalman', 100)
        self.pub_wheel = self.create_publisher(Float32MultiArray, 'wheels/kalman', 200)
        self.pub_ang = self.create_publisher(Float32MultiArray, 'glove/angles_kalman', 100)
        self.create_subscription(Imu, 'imu/raw', lambda m: self.on_imu(m, self.k_imu_r, self.pub_imu), 200)
        self.create_subscription(Imu, 'glove/imu/raw', self.on_glove_imu, 100)
        self.create_subscription(Float32MultiArray, 'wheels/speed', self.on_wheels, 200)
        self.create_subscription(Float32MultiArray, 'glove/angles', self.on_glove_angles, 100)
        self.get_logger().info('Filtros de Kalman listos: /imu/kalman, /wheels/kalman, '
                               '/glove/imu/kalman, /glove/angles_kalman')

    @staticmethod
    def filt_imu(m, ks):
        out = Imu()
        out.header = m.header
        a, w = m.linear_acceleration, m.angular_velocity
        out.linear_acceleration.x = ks[0].update(a.x)
        out.linear_acceleration.y = ks[1].update(a.y)
        out.linear_acceleration.z = ks[2].update(a.z)
        out.angular_velocity.x = ks[3].update(w.x)
        out.angular_velocity.y = ks[4].update(w.y)
        out.angular_velocity.z = ks[5].update(w.z)
        out.orientation_covariance[0] = -1.0
        return out

    def on_imu(self, m, ks, pub):
        pub.publish(self.filt_imu(m, ks))

    def on_wheels(self, m):
        if len(m.data) < 2:
            return
        wl = self.k_wheel[0].update(m.data[0], self.wheel_dt)
        wr = self.k_wheel[1].update(m.data[1], self.wheel_dt)
        self.pub_wheel.publish(Float32MultiArray(data=[wl, wr, self.k_wheel[0].a, self.k_wheel[1].a]))

    def on_glove_angles(self, m):
        if len(m.data) >= 4:               # [pitch, roll, pitch_raw, roll_raw]
            self.glove_raw_ang = (m.data[2], m.data[3])

    def on_glove_imu(self, m):
        self.pub_gimu.publish(self.filt_imu(m, self.k_imu_g))
        if self.glove_raw_ang is None:
            return
        now = time.monotonic()
        dt = 0.01 if self.glove_t is None else min(0.1, max(0.001, now - self.glove_t))
        self.glove_t = now
        # En el guante: gy gira alrededor de Y (cabeceo) y gx alrededor de X (alabeo)
        rate_p = m.angular_velocity.y * RAD2DEG
        rate_r = m.angular_velocity.x * RAD2DEG
        p = self.k_ang[0].update(self.glove_raw_ang[0], rate_p, dt)
        r = self.k_ang[1].update(self.glove_raw_ang[1], rate_r, dt)
        self.pub_ang.publish(Float32MultiArray(data=[p, r, self.k_ang[0].b, self.k_ang[1].b]))


def main():
    rclpy.init()
    node = KalmanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
