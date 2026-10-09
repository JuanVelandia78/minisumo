"""Guarda la telemetría del robot (/telemetry) en un archivo CSV."""
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String


class DataLogger(Node):
    def __init__(self):
        super().__init__('data_logger')
        self.declare_parameter('carpeta', '~/minisumo_datos')
        self.declare_parameter('nombre', 'prueba')
        carpeta = os.path.expanduser(self.get_parameter('carpeta').value)
        os.makedirs(carpeta, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.path = os.path.join(carpeta, f"{self.get_parameter('nombre').value}_{stamp}.csv")

        self.file = None
        self.pending = []
        self.count = 0

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, 'telemetry_header', self.on_header, latched)
        self.create_subscription(String, 'telemetry', self.on_line, 1000)
        self.create_timer(2.0, self.status)
        self.get_logger().info(f'Esperando la cabecera del robot_bridge... archivo: {self.path}')

    def on_header(self, msg):
        if self.file is not None:
            return
        self.file = open(self.path, 'w', buffering=1 << 16)
        self.file.write(msg.data + '\n')
        for line in self.pending:
            self.file.write(line + '\n')
        self.count += len(self.pending)
        self.pending.clear()
        self.get_logger().info(f'Guardando en {self.path}')

    def on_line(self, msg):
        if self.file is None:
            if len(self.pending) < 2000:
                self.pending.append(msg.data)
            return
        self.file.write(msg.data + '\n')
        self.count += 1

    def status(self):
        if self.file is not None:
            self.file.flush()
            self.get_logger().info(f'{self.count} muestras guardadas ({self.count / 200.0:.1f} s)')

    def close(self):
        if self.file is not None:
            self.file.close()
            self.get_logger().info(f'Archivo cerrado: {self.path} ({self.count} muestras)')


def main():
    rclpy.init()
    node = DataLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
