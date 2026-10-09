"""Puente UDP <-> ROS2 con el robot minisumo (ESP32-C3)."""
import math
import socket
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String, Float32MultiArray
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

G = 9.80665
DEG2RAD = math.pi / 180.0


class RobotBridge(Node):
    def __init__(self):
        super().__init__('robot_bridge')
        self.declare_parameter('robot_ip', '')          # vacío = detectar por broadcast
        self.declare_parameter('cmd_port', 8888)
        self.declare_parameter('beacon_port', 8889)
        self.declare_parameter('send_rate', 50.0)       # Hz de envío de comandos
        self.declare_parameter('cmd_timeout', 0.5)      # s sin /wheel_cmd -> detener
        self.declare_parameter('wheel_radius', 0.0325)
        self.declare_parameter('wheel_base', 0.085)

        self.robot_ip = self.get_parameter('robot_ip').value or None
        self.cmd_port = self.get_parameter('cmd_port').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value
        self.R = self.get_parameter('wheel_radius').value
        self.L = self.get_parameter('wheel_base').value

        # Socket de comandos: la telemetría vuelve a este mismo puerto
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('', 0))
        self.sock.setblocking(False)
        # Socket para escuchar el anuncio del robot
        self.beacon = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.beacon.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.beacon.bind(('', self.get_parameter('beacon_port').value))
        self.beacon.setblocking(False)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_header = self.create_publisher(String, 'telemetry_header', latched)
        self.pub_telem = self.create_publisher(String, 'telemetry', 100)
        self.pub_imu_raw = self.create_publisher(Imu, 'imu/raw', 50)
        self.pub_imu_f = self.create_publisher(Imu, 'imu/filtered', 50)
        self.pub_wheels = self.create_publisher(Float32MultiArray, 'wheels/speed', 50)
        self.pub_odom = self.create_publisher(Odometry, 'odom', 50)

        self.create_subscription(Float32MultiArray, 'wheel_cmd', self.on_wheel_cmd, 10)
        self.create_subscription(String, 'robot_cmd', self.on_robot_cmd, 10)

        self.wheel = (0.0, 0.0)
        self.wheel_t = 0.0
        self.cols = None
        self.idx = {}
        self.last_h = 0.0
        self.n_lines = 0

        self.create_timer(1.0 / self.get_parameter('send_rate').value, self.send_tick)
        self.create_timer(0.005, self.rx_tick)
        self.create_timer(2.0, self.status_tick)

        if self.robot_ip:
            self.get_logger().info(f'Robot en {self.robot_ip}:{self.cmd_port} (IP fija)')
        else:
            self.get_logger().info('Esperando el anuncio del robot en el puerto 8889...')

    # ---------------- envío ----------------
    def send(self, txt):
        if self.robot_ip:
            try:
                self.sock.sendto(txt.encode(), (self.robot_ip, self.cmd_port))
            except OSError as e:
                self.get_logger().warn(f'No se pudo enviar: {e}')

    def on_wheel_cmd(self, msg):
        if len(msg.data) >= 2:
            self.wheel = (float(msg.data[0]), float(msg.data[1]))
            self.wheel_t = time.monotonic()

    def on_robot_cmd(self, msg):
        self.send(msg.data.strip())
        self.get_logger().info(f'Comando enviado: {msg.data.strip()}')

    def send_tick(self):
        if not self.robot_ip:
            return
        now = time.monotonic()
        if self.cols is None and now - self.last_h > 1.0:
            self.send('H')
            self.last_h = now
        if now - self.wheel_t < self.cmd_timeout:
            self.send(f'W,{self.wheel[0]:.3f},{self.wheel[1]:.3f}')
        else:
            self.send('S')

    # ---------------- recepción ----------------
    def rx_tick(self):
        while True:
            try:
                data, _ = self.beacon.recvfrom(256)
            except BlockingIOError:
                break
            msg = data.decode(errors='ignore')
            if msg.startswith('MINISUMO,') and self.robot_ip is None:
                self.robot_ip = msg.split(',')[1]
                self.get_logger().info(f'Robot encontrado en {self.robot_ip}')
        while True:
            try:
                data, _ = self.sock.recvfrom(4096)
            except BlockingIOError:
                break
            for line in data.decode(errors='ignore').splitlines():
                self.handle_line(line)

    def handle_line(self, line):
        if line.startswith('H,'):
            self.cols = line[2:].split(',')
            self.idx = {c: i for i, c in enumerate(self.cols)}
            self.pub_header.publish(String(data=line[2:]))
            self.get_logger().info(f'Cabecera recibida ({len(self.cols)} columnas)')
            return
        if not line.startswith('D,') or self.cols is None:
            return
        vals = line[2:].split(',')
        if len(vals) != len(self.cols):
            return
        self.n_lines += 1
        self.pub_telem.publish(String(data=line[2:]))

        def f(c):
            return float(vals[self.idx[c]])

        stamp = self.get_clock().now().to_msg()
        self.pub_imu_raw.publish(self.make_imu(stamp, f, ''))
        self.pub_imu_f.publish(self.make_imu(stamp, f, '_f'))
        self.pub_wheels.publish(Float32MultiArray(
            data=[f('wl'), f('wr'), f('wl_f'), f('wr_f'), f('wl_ref'), f('wr_ref'), f('duty_l'), f('duty_r')]))

        od = Odometry()
        od.header.stamp = stamp
        od.header.frame_id = 'odom'
        od.child_frame_id = 'base_link'
        od.pose.pose.position.x = f('x')
        od.pose.pose.position.y = f('y')
        th = f('theta')
        od.pose.pose.orientation.z = math.sin(th / 2.0)
        od.pose.pose.orientation.w = math.cos(th / 2.0)
        od.twist.twist.linear.x = self.R * (f('wl_f') + f('wr_f')) / 2.0
        od.twist.twist.angular.z = f('gz_f') * DEG2RAD
        self.pub_odom.publish(od)

    @staticmethod
    def make_imu(stamp, f, suf):
        m = Imu()
        m.header.stamp = stamp
        m.header.frame_id = 'imu_link'
        m.linear_acceleration.x = f('ax' + suf) * G
        m.linear_acceleration.y = f('ay' + suf) * G
        m.linear_acceleration.z = f('az' + suf) * G
        m.angular_velocity.x = f('gx' + suf) * DEG2RAD
        m.angular_velocity.y = f('gy' + suf) * DEG2RAD
        m.angular_velocity.z = f('gz' + suf) * DEG2RAD
        m.orientation_covariance[0] = -1.0   # sin estimación de orientación
        return m

    def status_tick(self):
        if self.robot_ip:
            self.get_logger().info(f'Robot {self.robot_ip} | muestras recibidas: {self.n_lines / 2.0:.0f}/s')
        self.n_lines = 0


def main():
    rclpy.init()
    node = RobotBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.send('S')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
