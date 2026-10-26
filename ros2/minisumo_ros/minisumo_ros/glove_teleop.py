"""Teleoperación con el guante: inclinación de la mano -> /cmd_vel.

Escucha los paquetes UDP del guante (GL,t_ms,pitch,roll,pitch_raw,roll_raw) y publica:
  /cmd_vel        geometry_msgs/Twist   (lo convierte diff_kinematics en velocidad de cada rueda)
  /glove/angles   std_msgs/Float32MultiArray [pitch, roll, pitch_raw, roll_raw] en grados
"""
import math
import socket

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray


def shape(angle, deadzone, max_angle):
    """Ángulo en grados -> [-1, 1] con zona muerta (mano casi horizontal = quieto)."""
    a = abs(angle)
    if a < deadzone:
        return 0.0
    x = min((a - deadzone) / (max_angle - deadzone), 1.0)
    return x if angle > 0 else -x


class GloveTeleop(Node):
    def __init__(self):
        super().__init__('glove_teleop')
        self.declare_parameter('port', 8890)
        self.declare_parameter('v_max', 0.25)          # m/s con la mano totalmente inclinada
        self.declare_parameter('w_max', 4.0)           # rad/s de giro con la mano totalmente inclinada
        self.declare_parameter('deadzone_deg', 8.0)    # inclinación que se ignora
        self.declare_parameter('roll_deadzone_deg', 12.0)  # zona muerta del giro: en reposo la mano
                                                           # suele quedar algo ladeada
        self.declare_parameter('max_deg', 35.0)        # inclinación que da la velocidad máxima
        self.declare_parameter('invert_pitch', False)  # True si al inclinar adelante el robot retrocede
        self.declare_parameter('invert_roll', False)   # True si al inclinar a la derecha gira a la izquierda
        self.declare_parameter('swap_axes', False)     # True si el MPU quedó girado 90° en la mano
                                                       # (adelante gira y de lado avanza)
        self.declare_parameter('timeout', 0.5)         # s sin datos del guante -> detener

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('', self.get_parameter('port').value))
        self.sock.setblocking(False)

        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_ang = self.create_publisher(Float32MultiArray, 'glove/angles', 10)
        # Ángulos de la mano ya corregidos (swap/invert): [adelante, derecha] en grados, para el visualizador
        self.pub_hand = self.create_publisher(Float32MultiArray, 'glove/hand_angles', 10)

        self.pitch = 0.0
        self.roll = 0.0
        self.last_rx = None
        self.glove_addr = None
        self.stopped = True
        self.count = 0
        self.create_timer(0.02, self.loop)    # 50 Hz
        self.create_timer(0.5, self.hello)    # aviso al guante para que envíe directo (unicast)
        self.create_timer(2.0, self.report)
        self.get_logger().info(f'Esperando el guante en UDP {self.get_parameter("port").value}...')

    def loop(self):
        # Vacía el socket y se queda con el paquete más reciente
        while True:
            try:
                data, addr = self.sock.recvfrom(256)
            except BlockingIOError:
                break
            parts = data.decode(errors='ignore').strip().split(',')
            if len(parts) < 6 or parts[0] != 'GL':
                continue
            try:
                vals = [float(v) for v in parts[2:6]]
            except ValueError:
                continue
            if self.last_rx is None:
                self.get_logger().info(f'Guante encontrado en {addr[0]}')
            self.glove_addr = addr[0]
            if self.get_parameter('swap_axes').value:
                self.pitch, self.roll = vals[1], vals[0]
            else:
                self.pitch, self.roll = vals[0], vals[1]
            self.last_rx = self.get_clock().now()
            self.count += 1
            self.pub_ang.publish(Float32MultiArray(data=vals))

        now = self.get_clock().now()
        fresh = self.last_rx is not None and \
            (now - self.last_rx).nanoseconds * 1e-9 < self.get_parameter('timeout').value
        if not fresh:
            if not self.stopped:      # una orden de detener y luego silencio
                self.pub_cmd.publish(Twist())
                self.stopped = True
            return

        dz = self.get_parameter('deadzone_deg').value
        mx = self.get_parameter('max_deg').value
        sp = -1.0 if self.get_parameter('invert_pitch').value else 1.0
        sr = -1.0 if self.get_parameter('invert_roll').value else 1.0
        # Mano adelante = cabeceo negativo; mano a la derecha = alabeo positivo (con las correcciones)
        self.pub_hand.publish(Float32MultiArray(data=[-sp * self.pitch, sr * self.roll]))

        cmd = Twist()
        # Mano hacia adelante (cabeceo negativo) -> avanzar; mano a la derecha -> girar a la derecha
        cmd.linear.x = -sp * shape(self.pitch, dz, mx) * self.get_parameter('v_max').value
        # Con la mano muy inclinada adelante/atrás el alabeo pierde sentido (se acerca a la vertical):
        # su peso baja con el coseno del cabeceo y desaparece pasando 60°
        roll_w = max(0.0, (math.cos(math.radians(self.pitch)) - 0.5) / 0.5)
        rdz = self.get_parameter('roll_deadzone_deg').value
        cmd.angular.z = -sr * roll_w * shape(self.roll, rdz, mx) * self.get_parameter('w_max').value
        self.pub_cmd.publish(cmd)
        self.stopped = False

    def hello(self):
        # El broadcast del guante llega mal por un hotspot de celular: al recibir "GLP" el
        # guante pasa a enviar directo a este PC
        if self.glove_addr:
            try:
                self.sock.sendto(b'GLP', (self.glove_addr, self.get_parameter('port').value))
            except OSError:
                pass

    def report(self):
        if self.last_rx is None:
            return
        self.get_logger().info(
            f'Guante: {self.count / 2.0:.0f} paquetes/s | cabeceo {self.pitch:6.1f}° | alabeo {self.roll:6.1f}°')
        self.count = 0


def main():
    rclpy.init()
    node = GloveTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub_cmd.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
