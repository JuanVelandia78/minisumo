"""Visualización en RViz2 del robot sobre el dojo y de la mano del operador.

Suscribe:
  /odom               nav_msgs/Odometry           posición estimada del robot (robot_bridge)
  /wheels/speed       std_msgs/Float32MultiArray  [wl, wr, wl_f, wr_f, wl_ref, wr_ref, duty_l, duty_r]
  /telemetry(+header) std_msgs/String             batería, modo y muestras por segundo
  /glove/hand_angles  std_msgs/Float32MultiArray  [adelante, derecha] en grados (glove_teleop)
  /cmd_vel            geometry_msgs/Twist         orden de movimiento
Publica:
  TF odom -> base_link (robot), base_link -> rueda_izq / rueda_der (giran con los encoders)
     odom -> mano (mano del operador, al lado del dojo)
  /minisumo/markers   visualization_msgs/MarkerArray  modelo CAD, dojo, flechas, HUD y mano
  /minisumo/path      nav_msgs/Path                    trayectoria recorrida

Modelo 3D: meshes/*.stl, convertidos del ensamblaje de SolidWorks (m, X adelante, Z arriba,
origen entre las ruedas a nivel del piso). Con -p usar_cad:=false se dibuja un modelo simple.

El origen de odom es el punto donde arrancó el robot (o donde se reinició la odometría con R).
dojo_cx, dojo_cy: centro del dojo en ese marco (por defecto el robot arranca en el centro).
"""
import math
import re
import time
import unicodedata

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Twist, TransformStamped, PoseStamped, Point
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float32MultiArray, ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster

MESH = 'package://minisumo_ros/meshes/'
# (archivo, marco, color) del modelo CAD
CAD_PARTS = [
    ('chasis.stl', 'base_link', (0.10, 0.10, 0.11)),      # cuerpo, tapa y soporte de la PCB: negro
    ('soportes.stl', 'base_link', (0.82, 0.82, 0.85)),    # soportes de los motores: gris claro
    ('motores.stl', 'base_link', (0.62, 0.64, 0.68)),     # motorreductores: metálico
    ('rueda_izq.stl', 'rueda_izq', (0.98, 0.80, 0.08)),   # ruedas: amarillo
    ('rueda_der.stl', 'rueda_der', (0.98, 0.80, 0.08)),
]
ROBOT_ALTO = 0.082        # m, altura del modelo (para poner las flechas encima)


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


def speed_color(frac, a=1.0):
    """0 -> azul, 0,5 -> verde, 1 -> rojo."""
    f = max(0.0, min(1.0, frac))
    if f < 0.5:
        return color(0.1, 0.4 + 1.2 * f, 1.0 - 1.6 * f, a)
    return color(2.0 * (f - 0.5), 1.0 - 1.4 * (f - 0.5), 0.2 - 0.4 * (f - 0.5), a)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Visualizer(Node):
    def __init__(self):
        super().__init__('visualizer')
        self.declare_parameter('dojo_diametro', 0.77)   # m, minisumo
        self.declare_parameter('dojo_borde', 0.025)     # m, línea blanca del borde
        self.declare_parameter('dojo_cx', 0.0)          # m, centro del dojo en el marco odom
        self.declare_parameter('dojo_cy', 0.0)
        self.declare_parameter('lineas_salida', True)   # líneas de salida (shikiri) en el centro
        self.declare_parameter('aviso_borde', 0.08)     # m: avisa si el robot está a menos del borde
        self.declare_parameter('usar_cad', True)        # modelo de SolidWorks (False: modelo simple)
        self.declare_parameter('wheel_radius', 0.0162)
        self.declare_parameter('wheel_base', 0.079)     # m, entre centros de las ruedas (CAD)
        self.declare_parameter('omega_max', 13.5)       # rad/s de rueda: escala de colores y flechas
        self.declare_parameter('bat_baja', 7.0)         # V: batería 2S en amarillo por debajo
        self.declare_parameter('bat_critica', 6.6)      # V: en rojo por debajo
        self.declare_parameter('path_max', 3000)        # puntos de la trayectoria
        self.declare_parameter('texto_sin_espacios', True)  # RViz en VM dibuja el espacio enorme: usa '_'

        self.tf = TransformBroadcaster(self)
        self.pub_markers = self.create_publisher(MarkerArray, 'minisumo/markers', 10)
        self.pub_path = self.create_publisher(Path, 'minisumo/path', 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, 50)
        self.create_subscription(Float32MultiArray, 'wheels/speed', self.on_wheels, 200)
        self.create_subscription(Float32MultiArray, 'glove/hand_angles', self.on_hand, 20)
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd, 20)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, 'telemetry_header', self.on_header, latched)
        self.create_subscription(String, 'telemetry', self.on_telem, 500)

        self.pose = (0.0, 0.0, 0.0)
        self.v = 0.0                   # m/s medida
        self.w_meas = (0.0, 0.0)       # rad/s de cada rueda (filtradas)
        self.w_ref = (0.0, 0.0)        # rad/s pedidas
        self.wheel_ang = [0.0, 0.0]    # ángulo de cada rueda para animarlas
        self.t_wheels = None
        self.th_des = None             # rumbo deseado (integra el giro pedido)
        self.hand = None               # (adelante, derecha) en grados
        self.hand_t = None
        self.cmd = (0.0, 0.0)
        self.cmd_t = 0.0
        self.cols = None
        self.bat = None                # (V, A)
        self.mode = None
        self.n_telem = 0
        self.rate = 0.0
        self.t_telem = 0.0
        self.path = Path()
        self.path.header.frame_id = 'odom'
        self.trail = []                # (x, y, rapidez) para la trayectoria de colores

        self.create_timer(0.05, self.draw)      # 20 Hz
        self.create_timer(1.0, self.count_rate)
        self.get_logger().info('Visualizador listo: abre RViz2 con el marco fijo "odom"')

    # ---------------- entradas ----------------
    def on_odom(self, msg):
        p = msg.pose.pose
        th = 2.0 * math.atan2(p.orientation.z, p.orientation.w)
        x, y = p.position.x, p.position.y
        # La odometría se reinició (comando R): empieza una trayectoria nueva
        if math.hypot(x, y) < 0.005 and math.hypot(self.pose[0], self.pose[1]) > 0.03:
            self.path.poses.clear()
            self.trail.clear()
            self.th_des = None
        self.pose = (x, y, th)
        self.v = msg.twist.twist.linear.x

        last = self.path.poses[-1].pose.position if self.path.poses else None
        if last is None or math.hypot(x - last.x, y - last.y) > 0.005:
            ps = PoseStamped()
            ps.header = msg.header
            ps.header.frame_id = 'odom'
            ps.pose = p
            self.path.poses.append(ps)
            self.trail.append((x, y, abs(self.v)))
            if len(self.path.poses) > self.get_parameter('path_max').value:
                self.path.poses.pop(0)
                self.trail.pop(0)

    def on_wheels(self, msg):
        d = msg.data
        if len(d) < 6:
            return
        self.w_meas = (d[2], d[3])
        self.w_ref = (d[4], d[5])
        now = time.monotonic()
        dt = 0.0 if self.t_wheels is None else min(0.1, now - self.t_wheels)
        self.t_wheels = now
        self.wheel_ang[0] = wrap(self.wheel_ang[0] + d[2] * dt)
        self.wheel_ang[1] = wrap(self.wheel_ang[1] + d[3] * dt)
        # Rumbo deseado: con las ruedas detenidas es el actual; en marcha integra el giro pedido
        R, B = self.get_parameter('wheel_radius').value, self.get_parameter('wheel_base').value
        if abs(d[4]) < 1e-3 and abs(d[5]) < 1e-3:
            self.th_des = None
        else:
            if self.th_des is None:
                self.th_des = self.pose[2]
            self.th_des = wrap(self.th_des + R * (d[5] - d[4]) / B * dt)

    def on_header(self, msg):
        self.cols = {c: i for i, c in enumerate(msg.data.strip().split(','))}

    def on_telem(self, msg):
        self.n_telem += 1
        self.t_telem = time.monotonic()
        if self.cols is None or self.n_telem % 20:      # basta con 10 Hz para el HUD
            return
        v = msg.data.split(',')
        try:
            if 'v_bat' in self.cols:
                self.bat = (float(v[self.cols['v_bat']]), float(v[self.cols['i_bat']]))
            if 'mode' in self.cols:
                self.mode = int(float(v[self.cols['mode']]))
        except (ValueError, IndexError):
            pass

    def count_rate(self):
        self.rate = self.n_telem
        self.n_telem = 0

    def on_hand(self, msg):
        if len(msg.data) >= 2:
            self.hand = (float(msg.data[0]), float(msg.data[1]))
            self.hand_t = self.get_clock().now()

    def on_cmd(self, msg):
        self.cmd = (msg.linear.x, msg.angular.z)
        self.cmd_t = time.monotonic()

    # ---------------- ayudas ----------------
    def tf_msg(self, parent, child, x, y, z, q, stamp):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = float(z)
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = q
        return t

    def marker(self, ms, mid, ns, frame, mtype, stamp, rgba=(1, 1, 1, 1)):
        m = Marker()
        m.header.frame_id = frame
        m.header.stamp = stamp
        m.ns = ns
        m.id = mid
        m.type = mtype
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.color = rgba if isinstance(rgba, ColorRGBA) else color(*rgba)
        ms.markers.append(m)
        return m

    def hide(self, ms, mid, ns, stamp):
        m = Marker()
        m.header.frame_id = 'odom'
        m.header.stamp = stamp
        m.ns = ns
        m.id = mid
        m.action = Marker.DELETE
        ms.markers.append(m)

    def text(self, ms, mid, txt, x, y, z, size, rgba, stamp, frame='odom'):
        m = self.marker(ms, mid, 'hud', frame, Marker.TEXT_VIEW_FACING, stamp, rgba)
        m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
        m.scale.z = size
        # La fuente de RViz no trae tildes ni el símbolo de grados: se escriben en ASCII
        txt = unicodedata.normalize('NFKD', txt.replace('°', 'deg')).encode('ascii', 'ignore').decode()
        if self.get_parameter('texto_sin_espacios').value:
            txt = re.sub(r'([:,|]) +', r'\1', txt.strip())
            txt = re.sub(' +', '_', txt)
        m.text = txt or '.'
        return m

    def arrow(self, ms, mid, ns, frame, p0, p1, width, rgba, stamp):
        m = self.marker(ms, mid, ns, frame, Marker.ARROW, stamp, rgba)
        m.points = [Point(x=float(p0[0]), y=float(p0[1]), z=float(p0[2])),
                    Point(x=float(p1[0]), y=float(p1[1]), z=float(p1[2]))]
        m.scale.x, m.scale.y, m.scale.z = width, 2.2 * width, 2.5 * width
        return m

    # ---------------- dibujo ----------------
    def draw(self):
        now = self.get_clock().now()
        stamp = now.to_msg()
        x, y, th = self.pose
        g = lambda n: self.get_parameter(n).value
        D, borde = g('dojo_diametro'), g('dojo_borde')
        cx, cy = g('dojo_cx'), g('dojo_cy')
        R, B, wmax = g('wheel_radius'), g('wheel_base'), g('omega_max')

        hand_ok = self.hand is not None and (now - self.hand_t).nanoseconds < 1e9
        fwd, right = self.hand if hand_ok else (0.0, 0.0)
        hx, hy, hz = cx, cy - (D / 2 + 0.30), 0.12
        self.tf.sendTransform([
            self.tf_msg('odom', 'base_link', x, y, 0.0, quat_from_rpy(0, 0, th), stamp),
            self.tf_msg('base_link', 'rueda_izq', 0.0, B / 2, R, quat_from_rpy(0, self.wheel_ang[0], 0), stamp),
            self.tf_msg('base_link', 'rueda_der', 0.0, -B / 2, R, quat_from_rpy(0, self.wheel_ang[1], 0), stamp),
            self.tf_msg('odom', 'mano', hx, hy, hz,
                        quat_from_rpy(math.radians(right), math.radians(fwd), 0.0), stamp)])

        ms = MarkerArray()
        dist_borde = (D / 2 - borde) - math.hypot(x - cx, y - cy)
        cerca = dist_borde < g('aviso_borde')

        # --- Dojo: borde blanco (rojo si el robot está cerca), superficie negra y líneas de salida
        m = self.marker(ms, 0, 'dojo', 'odom', Marker.CYLINDER, stamp,
                        (0.95, 0.15, 0.1) if cerca else (0.95, 0.95, 0.95))
        m.pose.position.x, m.pose.position.y, m.pose.position.z = cx, cy, -0.006
        m.scale.x = m.scale.y = D
        m.scale.z = 0.004
        m = self.marker(ms, 1, 'dojo', 'odom', Marker.CYLINDER, stamp, (0.06, 0.06, 0.07))
        m.pose.position.x, m.pose.position.y, m.pose.position.z = cx, cy, -0.004
        m.scale.x = m.scale.y = D - 2 * borde
        m.scale.z = 0.004
        if g('lineas_salida'):
            for i, s in enumerate((1, -1)):
                m = self.marker(ms, 2 + i, 'dojo', 'odom', Marker.CUBE, stamp, (0.55, 0.32, 0.15))
                m.pose.position.x, m.pose.position.y, m.pose.position.z = cx + s * 0.06, cy, -0.0015
                m.scale.x, m.scale.y, m.scale.z = 0.01, 0.10, 0.001

        # --- Robot: modelo CAD (o simple) en base_link y en las ruedas
        if g('usar_cad'):
            for i, (fname, frame, rgb) in enumerate(CAD_PARTS):
                m = self.marker(ms, i, 'robot', frame, Marker.MESH_RESOURCE, stamp, rgb)
                m.mesh_resource = MESH + fname
                m.scale.x = m.scale.y = m.scale.z = 1.0
        else:
            m = self.marker(ms, 0, 'robot', 'base_link', Marker.CUBE, stamp, (0.1, 0.1, 0.11))
            m.pose.position.x, m.pose.position.z = 0.034, 0.045
            m.scale.x, m.scale.y, m.scale.z = 0.10, 0.10, 0.07
            for i, frame in enumerate(('rueda_izq', 'rueda_der')):
                m = self.marker(ms, 1 + i, 'robot', frame, Marker.CYLINDER, stamp, (0.98, 0.8, 0.08))
                m.pose.orientation.x = math.sin(math.pi / 4)      # eje del cilindro a lo largo de y
                m.pose.orientation.w = math.cos(math.pi / 4)
                m.scale.x = m.scale.y = 2 * R
                m.scale.z = 0.021

        # --- Velocidad de cada rueda: flecha al lado de la rueda, largo y color según la rapidez
        for i, (w, s) in enumerate(zip(self.w_meas, (1, -1))):
            yw = s * (B / 2 + 0.022)
            L = 0.10 * max(-1.0, min(1.0, w / wmax))
            if abs(L) > 0.004:
                self.arrow(ms, i, 'ruedas', 'base_link', (0, yw, R), (L, yw, R), 0.006,
                           speed_color(abs(w) / wmax), stamp)
            else:
                self.hide(ms, i, 'ruedas', stamp)

        # --- Rumbo real (blanco) y deseado (naranja) encima del robot
        zt = ROBOT_ALTO + 0.02
        self.arrow(ms, 0, 'rumbo', 'odom', (x, y, zt),
                   (x + 0.13 * math.cos(th), y + 0.13 * math.sin(th), zt), 0.008, (0.95, 0.95, 0.95), stamp)
        if self.th_des is not None:
            td = self.th_des
            self.arrow(ms, 1, 'rumbo', 'odom', (x, y, zt + 0.004),
                       (x + 0.13 * math.cos(td), y + 0.13 * math.sin(td), zt + 0.004), 0.006,
                       (1.0, 0.55, 0.0), stamp)
        else:
            self.hide(ms, 1, 'rumbo', stamp)

        # --- Trayectoria con color según la rapidez (azul lento -> rojo rápido)
        vmax = wmax * R
        if len(self.trail) >= 2:
            m = self.marker(ms, 0, 'trayectoria', 'odom', Marker.LINE_STRIP, stamp)
            m.scale.x = 0.006
            for px, py, sp in self.trail:
                m.points.append(Point(x=px, y=py, z=0.001))
                m.colors.append(speed_color(sp / vmax))
        else:
            self.hide(ms, 0, 'trayectoria', stamp)

        # --- Mano 3D: palma, cuatro dedos y pulgar, inclinada como la del operador
        skin = (0.95, 0.75, 0.6, 1.0) if hand_ok else (0.5, 0.5, 0.5, 0.5)
        m = self.marker(ms, 0, 'mano', 'mano', Marker.CUBE, stamp, skin)
        m.scale.x, m.scale.y, m.scale.z = 0.10, 0.085, 0.025
        for i in range(4):
            m = self.marker(ms, 1 + i, 'mano', 'mano', Marker.CUBE, stamp, skin)
            m.pose.position.x = 0.05 + 0.035 - (0.006 if i in (0, 3) else 0.0)
            m.pose.position.y = -0.03 + i * 0.02
            m.scale.x, m.scale.y, m.scale.z = 0.07 - (0.012 if i in (0, 3) else 0.0), 0.017, 0.018
        m = self.marker(ms, 5, 'mano', 'mano', Marker.CUBE, stamp, skin)
        m.pose.position.x, m.pose.position.y = 0.015, 0.055
        m.pose.orientation.z, m.pose.orientation.w = math.sin(math.radians(35) / 2), math.cos(math.radians(35) / 2)
        m.scale.x, m.scale.y, m.scale.z = 0.05, 0.02, 0.018
        m = self.marker(ms, 6, 'mano', 'mano', Marker.CYLINDER, stamp, (0.25, 0.3, 0.4, 1.0) if hand_ok else skin)
        m.pose.position.x = -0.10                                    # antebrazo con el guante
        m.pose.orientation.y, m.pose.orientation.w = math.sin(math.pi / 4), math.cos(math.pi / 4)
        m.scale.x = m.scale.y = 0.06
        m.scale.z = 0.11

        # --- Orden que envía la mano: flecha (avance) y arco (giro) en el piso junto a la mano
        v_cmd, w_cmd = self.cmd if time.monotonic() - self.cmd_t < 1.0 else (0.0, 0.0)
        vfrac = v_cmd / max(vmax, 1e-3)
        if abs(vfrac) > 0.02:
            self.arrow(ms, 0, 'orden', 'odom', (hx, hy, 0.002), (hx + 0.2 * vfrac, hy, 0.002), 0.012,
                       (0.2, 0.9, 0.3), stamp)
        else:
            self.hide(ms, 0, 'orden', stamp)
        ang = max(-1.0, min(1.0, w_cmd / 3.0)) * math.pi * 0.75
        if abs(ang) >= 0.05:
            m = self.marker(ms, 1, 'orden', 'odom', Marker.LINE_STRIP, stamp, (0.3, 0.7, 1.0))
            m.scale.x = 0.008
            n = max(2, int(abs(ang) / 0.1))
            for k in range(n + 1):        # arco desde el frente: izquierda (w > 0) antihorario
                a = ang * k / n
                m.points.append(Point(x=hx + 0.09 * math.cos(a), y=hy + 0.09 * math.sin(a), z=0.002))
        else:
            self.hide(ms, 1, 'orden', stamp)

        # --- HUD: textos fijos al lado del dojo
        hx2, hy2 = cx + D / 2 + 0.06, cy + 0.05
        ok = time.monotonic() - self.t_telem < 1.0
        modos = {0: 'detenido', 1: 'velocidad', 2: 'lazo abierto'}
        white, gray = (0.95, 0.95, 0.95), (0.65, 0.65, 0.65)
        c_ok = (0.3, 1.0, 0.4) if ok and self.rate > 100 else ((1.0, 0.8, 0.2) if ok else (1.0, 0.25, 0.2))
        if self.bat is not None and self.bat[0] > 3.0:
            vb, ib = self.bat
            c_bat = (1.0, 0.25, 0.2) if vb < g('bat_critica') else ((1.0, 0.8, 0.2) if vb < g('bat_baja')
                                                                  else (0.3, 1.0, 0.4))
            bat_txt = f'Bateria={vb:.2f}V,{ib * 1000:.0f}mA'
        else:
            c_bat, bat_txt = gray, 'Bateria: sin lectura'
        hud = [
            (f'ROBOT: conectado ({self.rate:.0f}/s)' if ok else 'ROBOT: SIN DATOS', c_ok, 0.034),
            (f'Modo: {modos.get(self.mode, "-")}', white, 0.028),
            (f'v={self.v * 100:.1f}cm/s', white, 0.028),
            (f'rumbo={math.degrees(th):.1f}°', white, 0.028),
            (f'izq={self.w_meas[0]:.1f}/{self.w_ref[0]:.1f}rad/s', white, 0.028),
            (f'der={self.w_meas[1]:.1f}/{self.w_ref[1]:.1f}rad/s', white, 0.028),
            (f'x={x * 100:.1f}cm, y={y * 100:.1f}cm', white, 0.028),
            (bat_txt, c_bat, 0.03),
            ('ruedas: medida/pedida', gray, 0.02),
            ('rumbo: blanco=real, naranja=deseado', gray, 0.02),
        ]
        z = 0.40
        for k, (txt, c, size) in enumerate(hud):
            self.text(ms, k, txt, hx2, hy2, z, size, c, stamp)
            z -= 1.45 * size
        if cerca:
            self.text(ms, 20, f'CERCA DEL BORDE ({max(0.0, dist_borde) * 100:.0f}cm)',
                      x, y, ROBOT_ALTO + 0.10, 0.04, (1.0, 0.25, 0.2), stamp)
        else:
            self.hide(ms, 20, 'hud', stamp)
        if hand_ok:
            self.text(ms, 21, f'Mano: adelante={fwd:.0f}°, derecha={right:.0f}°', hx, hy, hz + 0.16, 0.03, white, stamp)
            self.text(ms, 22, f'Orden: v={v_cmd * 100:.1f}cm/s, w={w_cmd:.2f}rad/s', hx, hy, hz + 0.115, 0.03,
                      white, stamp)
        else:
            self.text(ms, 21, 'Mano: sin datos del guante', hx, hy, hz + 0.14, 0.03, white, stamp)
            self.hide(ms, 22, 'hud', stamp)

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
