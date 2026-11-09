"""Ir a un punto del dojo: lleva el robot a una coordenada (x, y) corrigiendo con la odometría.

Marco: el de /odom. Con el robot en el centro del dojo mirando a +x y la odometría reiniciada
(comando R), (0, 0) es el centro del dojo y las coordenadas van en metros.

Control (uniciclo, lazo cerrado con /odom = encoders + giroscopio del robot):
  - error de rumbo grande (> giro_en_sitio): gira en el sitio hacia el punto
  - si no: avanza con v = kv·d (limitada y reducida por cos(error)) y corrige con w = kw·error
  - llega cuando la distancia es menor que tolerancia y se detiene

Suscribe:  /goal (geometry_msgs/Point, z se ignora)   /goal_cancel (std_msgs/Bool)
           /odom   /estop (std_msgs/Bool)
Publica:   /cmd_vel_auto (geometry_msgs/Twist) -> diff_kinematics le da prioridad sobre /cmd_vel
           /goal_state (std_msgs/String, JSON) estado para el panel

  ros2 run minisumo_ros goto_controller
  ros2 topic pub --once /goal geometry_msgs/msg/Point "{x: 0.15, y: 0.10}"
"""
import json
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class GotoController(Node):
    def __init__(self):
        super().__init__('goto_controller')
        par = {
            'v_max': 0.12,            # m/s
            'v_min': 0.035,           # m/s: por debajo los motores no arrancan bien
            'w_max': 2.5,             # rad/s
            'w_min': 0.9,             # rad/s mínima al girar en el sitio (zona muerta)
            'kv': 0.9,                # 1/s
            'kw': 3.0,                # 1/s
            'giro_en_sitio': 35.0,    # grados: error de rumbo a partir del cual gira sin avanzar
            'tolerancia': 0.02,       # m
            'radio_util': 0.31,       # m: radio del dojo (0,36 interior) menos medio robot
            'timeout': 20.0,          # s por objetivo
            'rate': 20.0,             # Hz
        }
        for k, v in par.items():
            self.declare_parameter(k, v)
        self.g = lambda n: self.get_parameter(n).value

        self.pose = None              # (x, y, th)
        self.goal = None              # (x, y)
        self.t_goal = 0.0
        self.state = 'libre'
        self.detail = ''
        self.estop = False
        self.moving = False

        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel_auto', 10)
        self.pub_state = self.create_publisher(String, 'goal_state', 10)
        self.create_subscription(Point, 'goal', self.on_goal, 10)
        self.create_subscription(Bool, 'goal_cancel', self.on_cancel, 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, 50)
        self.create_subscription(Bool, 'estop', self.on_estop, 10)
        self.create_timer(1.0 / self.g('rate'), self.tick)
        self.create_timer(0.25, self.publish_state)
        self.get_logger().info('Ir a punto listo: publica en /goal (m, origen = centro del dojo tras reiniciar odometría)')

    # ---------------- entradas ----------------
    def on_odom(self, m):
        p = m.pose.pose
        self.pose = (p.position.x, p.position.y, 2.0 * math.atan2(p.orientation.z, p.orientation.w))

    def on_estop(self, m):
        self.estop = bool(m.data)
        if self.estop and self.goal is not None:
            self.finish('cancelado', 'parada de emergencia')

    def on_cancel(self, m):
        if m.data and self.goal is not None:
            self.finish('cancelado', 'cancelado por el operador')

    def on_goal(self, m):
        r = math.hypot(m.x, m.y)
        rmax = self.g('radio_util')
        if r > rmax:
            self.state, self.detail = 'rechazado', f'fuera del área segura ({r * 100:.0f} cm > {rmax * 100:.0f} cm del centro)'
            self.get_logger().warning(f'Objetivo rechazado: {self.detail}')
            self.publish_state()
            return
        if self.estop:
            self.state, self.detail = 'rechazado', 'parada de emergencia activa'
            self.publish_state()
            return
        self.goal = (m.x, m.y)
        self.t_goal = time.monotonic()
        self.state, self.detail = 'yendo', ''
        self.get_logger().info(f'Nuevo objetivo: x={m.x * 100:.1f} cm, y={m.y * 100:.1f} cm')
        self.publish_state()

    # ---------------- control ----------------
    def finish(self, state, detail=''):
        self.goal = None
        self.state, self.detail = state, detail
        self.stop()
        self.get_logger().info(f'Objetivo {state}{": " + detail if detail else ""}')
        self.publish_state()

    def stop(self):
        if self.moving:
            for _ in range(3):
                self.pub_cmd.publish(Twist())
            self.moving = False

    def tick(self):
        if self.goal is None or self.pose is None:
            return
        if time.monotonic() - self.t_goal > self.g('timeout'):
            self.finish('cancelado', 'tiempo agotado')
            return
        x, y, th = self.pose
        dx, dy = self.goal[0] - x, self.goal[1] - y
        d = math.hypot(dx, dy)
        if d < self.g('tolerancia'):
            self.finish('llegó', f'error {d * 100:.1f} cm'.replace('.', ','))
            return
        e = wrap(math.atan2(dy, dx) - th)
        cmd = Twist()
        if abs(e) > math.radians(self.g('giro_en_sitio')):
            w = self.g('kw') * e
            w = math.copysign(min(self.g('w_max'), max(self.g('w_min'), abs(w))), e)
            cmd.angular.z = w
        else:
            v = min(self.g('v_max'), self.g('kv') * d) * max(0.0, math.cos(e))
            cmd.linear.x = max(self.g('v_min'), v)
            cmd.angular.z = max(-self.g('w_max'), min(self.g('w_max'), self.g('kw') * e))
        self.pub_cmd.publish(cmd)
        self.moving = True
        self.dist = d

    def publish_state(self):
        d = None
        if self.goal is not None and self.pose is not None:
            d = math.hypot(self.goal[0] - self.pose[0], self.goal[1] - self.pose[1])
        self.pub_state.publish(String(data=json.dumps({
            'state': self.state, 'detail': self.detail,
            'goal': list(self.goal) if self.goal else None, 'dist': d,
            'radio': self.g('radio_util')})))


def main():
    rclpy.init()
    node = GotoController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
