"""Visualización en RViz2 del robot sobre el dojo y de la mano del operador.

Suscribe:
  /odom               nav_msgs/Odometry           posición estimada del robot (robot_bridge)
  /glove/hand_angles  std_msgs/Float32MultiArray  [adelante, derecha] en grados (glove_teleop)
  /cmd_vel            geometry_msgs/Twist         orden de movimiento
Publica:
  TF odom -> base_link (robot) y odom -> mano (mano del operador, al lado del dojo)
  /minisumo/markers   visualization_msgs/MarkerArray  dojo, robot, mano y textos
  /minisumo/path      nav_msgs/Path                    trayectoria recorrida

El origen de odom es el punto donde arrancó el robot (o donde se reinició la odometría con R).
dojo_cx, dojo_cy: centro del dojo en ese marco (por defecto el robot arranca en el centro).
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, PoseStamped, Point
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float32MultiArray, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster


def quat_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy,     # x
            cr * sp * cy + sr * cp * sy,     # y
            cr * cp * sy - sr * sp * cy,     # z
            cr * cp * cy + sr * sp * sy)     # w


def color(r, g, b, a=1.0):
    return ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))


class Visualizer(Node):
    def __init__(self):
        super().__init__('visualizer')
        self.declare_parameter('dojo_diametro', 0.77)   # m, minisumo
        self.declare_parameter('dojo_borde', 0.025)     # m, línea blanca del borde
        self.declare_parameter('dojo_cx', 0.0)          # m, centro del dojo en el marco odom
        self.declare_parameter('dojo_cy', 0.0)
        self.declare_parameter('robot_largo', 0.10)     # m
        self.declare_parameter('robot_ancho', 0.10)
        self.declare_parameter('wheel_radius', 0.0325)
        self.declare_parameter('wheel_base', 0.085)
        self.declare_parameter('path_max', 3000)        # puntos de la trayectoria

        self.tf = TransformBroadcaster(self)
        self.pub_markers = self.create_publisher(MarkerArray, 'minisumo/markers', 10)
        self.pub_path = self.create_publisher(Path, 'minisumo/path', 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, 50)
        self.create_subscription(Float32MultiArray, 'glove/hand_angles', self.on_hand, 20)
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd, 20)

        self.pose = (0.0, 0.0, 0.0)
        self.hand = None              # (adelante, derecha) en grados
        self.hand_t = None
        self.cmd = (0.0, 0.0)
        self.path = Path()
        self.path.header.frame_id = 'odom'

        self.create_timer(0.05, self.draw)      # 20 Hz
        self.get_logger().info('Visualizador listo: abre RViz2 con el marco fijo "odom"')

    # ---------------- entradas ----------------
    def on_odom(self, msg):
        p = msg.pose.pose
        th = 2.0 * math.atan2(p.orientation.z, p.orientation.w)
        x, y = p.position.x, p.position.y
        # La odometría se reinició (comando R): empieza una trayectoria nueva
        if math.hypot(x, y) < 0.005 and math.hypot(self.pose[0], self.pose[1]) > 0.03:
            self.path.poses.clear()
        self.pose = (x, y, th)

        last = self.path.poses[-1].pose.position if self.path.poses else None
        if last is None or math.hypot(x - last.x, y - last.y) > 0.005:
            ps = PoseStamped()
            ps.header = msg.header
            ps.header.frame_id = 'odom'
            ps.pose = p
            self.path.poses.append(ps)
            if len(self.path.poses) > self.get_parameter('path_max').value:
                self.path.poses.pop(0)

    def on_hand(self, msg):
        if len(msg.data) >= 2:
            self.hand = (float(msg.data[0]), float(msg.data[1]))
            self.hand_t = self.get_clock().now()

    def on_cmd(self, msg):
        self.cmd = (msg.linear.x, msg.angular.z)

    # ---------------- dibujo ----------------
    def tf_msg(self, child, x, y, z, q, stamp):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = child
        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = float(z)
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = q
        return t

    def marker(self, mid, ns, frame, mtype, stamp):
        m = Marker()
        m.header.frame_id = frame
        m.header.stamp = stamp
        m.ns = ns
        m.id = mid
        m.type = mtype
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        return m

    def draw(self):
        now = self.get_clock().now()
        stamp = now.to_msg()
        x, y, th = self.pose
        g = lambda n: self.get_parameter(n).value
        D, borde = g('dojo_diametro'), g('dojo_borde')
        cx, cy = g('dojo_cx'), g('dojo_cy')
        L, W, R, B = g('robot_largo'), g('robot_ancho'), g('wheel_radius'), g('wheel_base')

        # Mano: al lado del dojo, inclinada como la del operador
        hand_ok = self.hand is not None and (now - self.hand_t).nanoseconds < 1e9
        fwd, right = self.hand if hand_ok else (0.0, 0.0)
        hx, hy, hz = cx, cy - (D / 2 + 0.35), 0.15
        tfs = [self.tf_msg('base_link', x, y, 0.0, quat_from_rpy(0, 0, th), stamp),
               self.tf_msg('mano', hx, hy, hz,
                           quat_from_rpy(math.radians(right), math.radians(fwd), 0.0), stamp)]
        self.tf.sendTransform(tfs)

        ms = MarkerArray()
        # Dojo: disco blanco (borde) y disco negro encima
        m = self.marker(0, 'dojo', 'odom', Marker.CYLINDER, stamp)
        m.pose.position.x, m.pose.position.y, m.pose.position.z = cx, cy, -0.006
        m.scale.x = m.scale.y = D
        m.scale.z = 0.004
        m.color = color(0.95, 0.95, 0.95)
        ms.markers.append(m)
        m = self.marker(1, 'dojo', 'odom', Marker.CYLINDER, stamp)
        m.pose.position.x, m.pose.position.y, m.pose.position.z = cx, cy, -0.003
        m.scale.x = m.scale.y = D - 2 * borde
        m.scale.z = 0.004
        m.color = color(0.08, 0.08, 0.08)
        ms.markers.append(m)

        # Robot: chasis, ruedas y flecha hacia el frente
        m = self.marker(0, 'robot', 'base_link', Marker.CUBE, stamp)
        m.pose.position.z = 0.025
        m.scale.x, m.scale.y, m.scale.z = L, W, 0.03
        m.color = color(0.85, 0.15, 0.15)
        ms.markers.append(m)
        for i, side in enumerate((1, -1)):
            m = self.marker(1 + i, 'robot', 'base_link', Marker.CYLINDER, stamp)
            m.pose.position.y = side * B / 2
            m.pose.position.z = R
            m.pose.orientation.x = math.sin(math.pi / 4)     # eje del cilindro a lo largo de y
            m.pose.orientation.w = math.cos(math.pi / 4)
            m.scale.x = m.scale.y = 2 * R
            m.scale.z = 0.02
            m.color = color(0.2, 0.2, 0.2)
            ms.markers.append(m)
        m = self.marker(3, 'robot', 'base_link', Marker.ARROW, stamp)
        m.points = [Point(x=0.0, y=0.0, z=0.045), Point(x=L * 0.9, y=0.0, z=0.045)]
        m.scale.x, m.scale.y, m.scale.z = 0.012, 0.025, 0.025
        m.color = color(1.0, 0.85, 0.1)
        ms.markers.append(m)

        # Mano: palma y flecha que apunta hacia adelante
        m = self.marker(0, 'mano', 'mano', Marker.CUBE, stamp)
        m.scale.x, m.scale.y, m.scale.z = 0.12, 0.09, 0.02
        m.color = color(0.95, 0.75, 0.6) if hand_ok else color(0.5, 0.5, 0.5, 0.5)
        ms.markers.append(m)
        m = self.marker(1, 'mano', 'mano', Marker.ARROW, stamp)
        m.points = [Point(x=0.0, y=0.0, z=0.012), Point(x=0.1, y=0.0, z=0.012)]
        m.scale.x, m.scale.y, m.scale.z = 0.01, 0.02, 0.02
        m.color = color(0.1, 0.6, 1.0)
        ms.markers.append(m)

        # Textos
        m = self.marker(0, 'texto', 'odom', Marker.TEXT_VIEW_FACING, stamp)
        m.pose.position.x, m.pose.position.y, m.pose.position.z = cx + D / 2 + 0.05, cy, 0.25
        m.scale.z = 0.045
        m.color = color(1, 1, 1)
        m.text = (f'Robot  x {x * 100:5.1f} cm   y {y * 100:5.1f} cm   th {math.degrees(th):5.1f} deg\n'
                  f'Orden  v {self.cmd[0]:4.2f} m/s   w {self.cmd[1]:4.2f} rad/s')
        ms.markers.append(m)
        m = self.marker(1, 'texto', 'odom', Marker.TEXT_VIEW_FACING, stamp)
        m.pose.position.x, m.pose.position.y, m.pose.position.z = hx, hy, hz + 0.12
        m.scale.z = 0.04
        m.color = color(1, 1, 1)
        m.text = (f'Mano  adelante {fwd:5.1f} deg  derecha {right:5.1f} deg' if hand_ok
                  else 'Mano: sin datos del guante')
        ms.markers.append(m)

        self.pub_markers.publish(ms)
        self.path.header.stamp = stamp
        self.pub_path.publish(self.path)


def main():
    rclpy.init()
    node = Visualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
