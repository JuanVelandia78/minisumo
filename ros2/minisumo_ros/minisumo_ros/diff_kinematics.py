"""Modelo cinemático del robot diferencial: (v, w) -> /wheel_cmd (wl, wr) en rad/s.

Dos entradas con prioridad:
  /cmd_vel_auto  control automático (ir a un punto): manda mientras esté llegando
  /cmd_vel       guante, pad manual del panel, rutinas: se ignora si /cmd_vel_auto está activo
"""
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray


class DiffKinematics(Node):
    def __init__(self):
        super().__init__('diff_kinematics')
        self.declare_parameter('wheel_radius', 0.0162)   # m
        self.declare_parameter('wheel_base', 0.079)      # m, entre centros de las ruedas
        self.declare_parameter('omega_max', 12.0)        # rad/s máximos por rueda (techo medido ~13,5)
        self.declare_parameter('auto_hold', 0.3)         # s que el control automático conserva la prioridad
        self.R = self.get_parameter('wheel_radius').value
        self.L = self.get_parameter('wheel_base').value
        self.wmax = self.get_parameter('omega_max').value
        self.t_auto = 0.0

        self.pub = self.create_publisher(Float32MultiArray, 'wheel_cmd', 10)
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)
        self.create_subscription(Twist, 'cmd_vel_auto', self.on_cmd_auto, 10)
        self.get_logger().info(f'R={self.R} m, L={self.L} m, omega_max={self.wmax} rad/s')

    def on_cmd_auto(self, msg):
        self.t_auto = time.monotonic()
        self.publish(msg.linear.x, msg.angular.z)

    def on_cmd_vel(self, msg):
        if time.monotonic() - self.t_auto < self.get_parameter('auto_hold').value:
            return                           # el control automático tiene prioridad
        self.publish(msg.linear.x, msg.angular.z)

    def publish(self, v, w):
        wl = (v - w * self.L / 2.0) / self.R
        wr = (v + w * self.L / 2.0) / self.R
        peak = max(abs(wl), abs(wr))
        if peak > self.wmax:                 # escala ambas ruedas: conserva el radio de giro
            k = self.wmax / peak
            wl *= k
            wr *= k
        self.pub.publish(Float32MultiArray(data=[wl, wr]))


def main():
    rclpy.init()
    node = DiffKinematics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
