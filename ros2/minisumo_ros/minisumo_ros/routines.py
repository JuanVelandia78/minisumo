"""Rutinas de movimiento con velocidad y tiempo conocidos (experimento de posición final).

Para cada iteración:
  1. Reinicia la odometría del robot (comando R) y espera a que esté quieto.
  2. Ejecuta la rutina publicando /cmd_vel por tramos (v, w, duración).
  3. Detiene el robot y compara:
       - posición DESEADA: integración exacta del modelo cinemático con los tramos ordenados
       - posición ESTIMADA: última /odom (encoders + giroscopio fusionados en el robot)
  4. Agrega una fila a ~/minisumo_datos/rutinas.csv. La posición REAL se mide con cinta
     y se escribe a mano en las columnas x_real, y_real.

Rutinas: recta, giro, cuadrado, arco.
Ejemplo:
  ros2 run minisumo_ros routines --ros-args -p rutina:=cuadrado -p iteraciones:=3
"""
import math
import os
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String


def build_segments(rutina, v, w, distancia, angulo_deg, radio):
    """Devuelve la lista de tramos (v [m/s], w [rad/s], duración [s])."""
    pausa = (0.0, 0.0, 0.4)
    ang = math.radians(angulo_deg)
    if rutina == 'recta':
        return [(v, 0.0, distancia / v)]
    if rutina == 'giro':
        return [(0.0, math.copysign(w, ang), abs(ang) / w)]
    if rutina == 'cuadrado':
        segs = []
        for _ in range(4):
            segs += [(v, 0.0, distancia / v), pausa, (0.0, w, (math.pi / 2) / w), pausa]
        return segs
    if rutina == 'arco':
        # cuarto de circunferencia hacia la izquierda con el radio dado
        return [(v, v / radio, (math.pi / 2) * radio / v)]
    raise ValueError(f'Rutina desconocida: {rutina}')


def integrate_model(segments):
    """Integración exacta del modelo del robot diferencial (uniciclo) para cada tramo."""
    x = y = th = 0.0
    for v, w, T in segments:
        if abs(w) < 1e-9:
            x += v * T * math.cos(th)
            y += v * T * math.sin(th)
        else:
            x += (v / w) * (math.sin(th + w * T) - math.sin(th))
            y += -(v / w) * (math.cos(th + w * T) - math.cos(th))
            th += w * T
    return x, y, math.atan2(math.sin(th), math.cos(th))


class Routines(Node):
    def __init__(self):
        super().__init__('routines')
        self.declare_parameter('rutina', 'recta')
        self.declare_parameter('iteraciones', 3)
        self.declare_parameter('v', 0.15)            # m/s en los tramos rectos
        self.declare_parameter('w', 3.0)             # rad/s en los giros en el sitio
        self.declare_parameter('distancia', 0.5)     # m (recta) o lado del cuadrado
        self.declare_parameter('angulo', 90.0)       # grados (giro)
        self.declare_parameter('radio', 0.2)         # m (arco)
        self.declare_parameter('carpeta', '~/minisumo_datos')

        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_robot = self.create_publisher(String, 'robot_cmd', 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, 50)
        self.odom = None

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        self.odom = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                     2.0 * math.atan2(q.z, q.w))

    def wait(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.01)

    def drive(self, v, w, seconds):
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        end = time.monotonic() + seconds
        next_pub = 0.0
        while time.monotonic() < end:
            now = time.monotonic()
            if now >= next_pub:
                self.pub_cmd.publish(cmd)
                next_pub = now + 0.05            # 20 Hz
            rclpy.spin_once(self, timeout_sec=0.005)

    def stop(self):
        for _ in range(5):
            self.pub_cmd.publish(Twist())
            self.wait(0.05)

    def run(self):
        p = {n: self.get_parameter(n).value for n in
             ('rutina', 'iteraciones', 'v', 'w', 'distancia', 'angulo', 'radio', 'carpeta')}
        segs = build_segments(p['rutina'], p['v'], p['w'], p['distancia'], p['angulo'], p['radio'])
        xd, yd, thd = integrate_model(segs)
        total = sum(s[2] for s in segs)

        carpeta = os.path.expanduser(p['carpeta'])
        os.makedirs(carpeta, exist_ok=True)
        path = os.path.join(carpeta, 'rutinas.csv')
        new = not os.path.exists(path)
        f = open(path, 'a')
        if new:
            f.write('fecha,rutina,iteracion,x_deseada,y_deseada,theta_deseada,'
                    'x_estimada,y_estimada,theta_estimada,x_real,y_real\n')

        self.get_logger().info(
            f"Rutina '{p['rutina']}': {len(segs)} tramos, {total:.1f} s | "
            f"posición deseada x={xd:.3f} m, y={yd:.3f} m, theta={math.degrees(thd):.1f}°")
        self.wait(1.0)   # deja que se conecten los publicadores

        for it in range(1, p['iteraciones'] + 1):
            input(f'\nIteración {it}: coloca el robot en el origen mirando al eje x y presiona Enter...')
            self.pub_robot.publish(String(data='R'))     # reinicia odometría y encoders
            self.wait(1.5)
            for v, w, T in segs:
                self.drive(v, w, T)
            self.stop()
            self.wait(1.0)                                # el robot termina de detenerse

            xe, ye, the = self.odom if self.odom else (float('nan'),) * 3
            self.get_logger().info(
                f'Iteración {it}: deseada ({xd:.3f}, {yd:.3f}) m | estimada ({xe:.3f}, {ye:.3f}) m, '
                f'theta {math.degrees(the):.1f}° -> mide la posición real con cinta')
            f.write(f"{datetime.now():%Y-%m-%d %H:%M:%S},{p['rutina']},{it},"
                    f"{xd:.4f},{yd:.4f},{thd:.4f},{xe:.4f},{ye:.4f},{the:.4f},,\n")
            f.flush()

        f.close()
        self.get_logger().info(f'Resultados agregados a {path} (completa x_real, y_real)')


def main():
    rclpy.init()
    node = Routines()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
