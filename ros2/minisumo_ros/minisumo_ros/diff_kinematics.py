"""Modelo cinemático del robot diferencial: /cmd_vel (v, w) -> /wheel_cmd (wl, wr) en rad/s."""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray


class DiffKinematics(Node):
    def __init__(self):
        super().__init__('diff_kinematics')
        self.declare_parameter('wheel_radius', 0.0325)   # m
        self.declare_parameter('wheel_base', 0.085)      # m, entre centros de las ruedas
        self.declare_parameter('omega_max', 18.0)        # rad/s máximos por rueda
        self.R = self.get_parameter('wheel_radius').value
        self.L = self.get_parameter('wheel_base').value
        self.wmax = self.get_parameter('omega_max').value

        self.pub = self.create_publisher(Float32MultiArray, 'wheel_cmd', 10)
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)
        self.get_logger().info(f'R={self.R} m, L={self.L} m, omega_max={self.wmax} rad/s')

    def on_cmd_vel(self, msg):
        v = msg.linear.x
        w = msg.angular.z
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
