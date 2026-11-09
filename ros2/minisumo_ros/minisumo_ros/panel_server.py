"""Panel web de CAR-LOSS: sirve la página y la conecta con ROS2 por WebSocket.

Un solo puerto (por defecto 8080) y solo la biblioteca estándar de Python: no hace falta
rosbridge ni instalar nada con sudo. Abre http://<IP de la VM>:8080 en cualquier navegador
de la misma red (la VM, Windows o el celular).

Hacia el navegador (JSON, ~25 Hz):
  estado del robot, odometría, ruedas, batería, guante, enlaces y muestras nuevas de las
  señales crudas / Kalman / filtro del ESP32 para las gráficas.
Desde el navegador:
  {"t":"cmd","v":"R"}             comando al robot (/robot_cmd): S R C T G F Y
  {"t":"estop","v":true|false}    parada de emergencia (/estop, la respeta robot_bridge)
  {"t":"drive","v":0.1,"w":0.5}   manejo manual (/cmd_vel), solo mientras el panel lo envíe

  ros2 run minisumo_ros panel_server
  ros2 run minisumo_ros panel_server --ros-args -p port:=8080
"""
import asyncio
import base64
import collections
import hashlib
import json
import math
import mimetypes
import os
import socket
import struct
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float32MultiArray, String

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:          # pragma: no cover
    get_package_share_directory = None

WS_GUID = '258EAFA5-E914-47DA-95CA-C5AB0DC85B11'
RAD2DEG = 180.0 / math.pi
ALLOWED_CMDS = ('S', 'R', 'C', 'T', 'G,', 'F,', 'Y,', 'P')
# Series de las gráficas: nombre -> se guardan (t, valor) a la tasa nativa (diezmadas a ~100 Hz)
SERIES = ('gz_raw', 'gz_k', 'gz_fw', 'ax_raw', 'ax_k', 'ax_fw',
          'wl_raw', 'wl_k', 'wl_fw', 'wr_raw', 'wr_k', 'wr_fw',
          'gp_raw', 'gp_k', 'gp_fw', 'gr_raw', 'gr_k', 'gr_fw',
          'ggy_raw', 'ggy_k', 'ggy_fw')


def lan_ips():
    """IPv4 de esta máquina en la red local (sin loopback), para abrir el panel desde otro equipo."""
    ips = []
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('10.255.255.255', 1))      # no envía nada: solo elige la interfaz de salida
        ips.append(s.getsockname()[0])
        s.close()
    except OSError:
        pass
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            ip = info[4][0]
            if not ip.startswith('127.') and ip not in ips:
                ips.append(ip)
    except OSError:
        pass
    return ips


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class PanelNode(Node):
    def __init__(self):
        super().__init__('panel_server')
        self.declare_parameter('port', 8080)
        self.declare_parameter('rate', 25.0)            # Hz de envío al navegador
        self.declare_parameter('wheel_radius', 0.0162)
        self.declare_parameter('wheel_base', 0.079)
        self.declare_parameter('web_dir', '')           # vacío: share/minisumo_ros/web

        self.lock = threading.Lock()
        self.t0 = time.monotonic()
        self.cols = None
        self.st = {
            'x': 0.0, 'y': 0.0, 'th': 0.0, 'v': 0.0, 'w': 0.0, 'th_des': None,
            'wl': 0.0, 'wr': 0.0, 'wl_ref': 0.0, 'wr_ref': 0.0, 'duty_l': 0.0, 'duty_r': 0.0,
            'mode': None, 'v_bat': None, 'i_bat': None, 'p_bat': None, 'ticks_l': 0, 'ticks_r': 0,
            'gz': 0.0, 'hand': None, 'cmd_v': 0.0, 'cmd_w': 0.0, 'estop': False,
        }
        self.t_last = {'robot': 0.0, 'glove': 0.0, 'cmd': 0.0}
        self.ips = lan_ips()
        self.t_ips = time.monotonic()
        self.count = {'robot': 0, 'glove': 0}
        self.rate = {'robot': 0.0, 'glove': 0.0}
        self.buf = {k: collections.deque(maxlen=4000) for k in SERIES}
        self.dec = collections.Counter()
        self.t_wheels = None
        self.drive = None        # (v, w, t) manejo manual desde el panel

        self.pub_cmd = self.create_publisher(String, 'robot_cmd', 10)
        self.pub_estop = self.create_publisher(Bool, 'estop', 10)
        self.pub_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_goal = self.create_publisher(Point, 'goal', 10)
        self.pub_goal_cancel = self.create_publisher(Bool, 'goal_cancel', 10)
        self.goal_state = None
        self.create_subscription(String, 'goal_state', self.on_goal_state, 10)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, 'telemetry_header', self.on_header, latched)
        self.create_subscription(String, 'telemetry', self.on_telem, 500)
        self.create_subscription(Odometry, 'odom', self.on_odom, 100)
        self.create_subscription(Float32MultiArray, 'wheels/speed', self.on_wheels, 200)
        self.create_subscription(Float32MultiArray, 'wheels/kalman', self.on_wheels_k, 200)
        self.create_subscription(Imu, 'imu/raw', lambda m: self.on_imu(m, 'raw'), 200)
        self.create_subscription(Imu, 'imu/kalman', lambda m: self.on_imu(m, 'k'), 200)
        self.create_subscription(Imu, 'imu/filtered', lambda m: self.on_imu(m, 'fw'), 200)
        self.create_subscription(Float32MultiArray, 'glove/angles', self.on_glove_angles, 100)
        self.create_subscription(Float32MultiArray, 'glove/angles_kalman', self.on_glove_angles_k, 100)
        self.create_subscription(Float32MultiArray, 'glove/hand_angles', self.on_hand, 50)
        self.create_subscription(Imu, 'glove/imu/raw', lambda m: self.on_gimu(m, 'raw'), 100)
        self.create_subscription(Imu, 'glove/imu/kalman', lambda m: self.on_gimu(m, 'k'), 100)
        self.create_subscription(Imu, 'glove/imu/filtered', lambda m: self.on_gimu(m, 'fw'), 100)
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 50)
        self.create_timer(1.0, self.count_rates)
        self.create_timer(0.05, self.drive_tick)
        self.create_timer(1.0, self.estop_tick)

    # ---------------- utilidades ----------------
    def now(self):
        return time.monotonic() - self.t0

    def push(self, key, value, every=1):
        self.dec[key] += 1
        if self.dec[key] % every:
            return
        with self.lock:
            self.buf[key].append((round(self.now(), 3), round(float(value), 4)))

    # ---------------- entradas ROS ----------------
    def on_header(self, msg):
        self.cols = {c: i for i, c in enumerate(msg.data.strip().split(','))}

    def on_telem(self, msg):
        self.count['robot'] += 1
        self.t_last['robot'] = time.monotonic()
        if self.cols is None or self.count['robot'] % 10:
            return
        v = msg.data.split(',')
        c = self.cols

        def f(k):
            return float(v[c[k]]) if k in c else None
        try:
            with self.lock:
                for k in ('v_bat', 'i_bat', 'p_bat', 'duty_l', 'duty_r'):
                    if k in c:
                        self.st[k] = f(k)
                if 'mode' in c:
                    self.st['mode'] = int(f('mode'))
                if 'ticks_l' in c:
                    self.st['ticks_l'], self.st['ticks_r'] = int(f('ticks_l')), int(f('ticks_r'))
        except (ValueError, IndexError):
            pass

    def on_odom(self, msg):
        p = msg.pose.pose
        with self.lock:
            self.st['x'], self.st['y'] = p.position.x, p.position.y
            self.st['th'] = 2.0 * math.atan2(p.orientation.z, p.orientation.w)
            self.st['v'] = msg.twist.twist.linear.x
            self.st['w'] = msg.twist.twist.angular.z

    def on_wheels(self, msg):
        d = msg.data
        if len(d) < 8:
            return
        now = time.monotonic()
        dt = 0.0 if self.t_wheels is None else min(0.1, now - self.t_wheels)
        self.t_wheels = now
        R = self.get_parameter('wheel_radius').value
        B = self.get_parameter('wheel_base').value
        with self.lock:
            s = self.st
            s['wl'], s['wr'], s['wl_ref'], s['wr_ref'] = d[2], d[3], d[4], d[5]
            s['duty_l'], s['duty_r'] = d[6], d[7]
            # Rumbo deseado: con las ruedas detenidas es el actual; en marcha integra el giro pedido
            if abs(d[4]) < 1e-3 and abs(d[5]) < 1e-3:
                s['th_des'] = None
            else:
                base = s['th'] if s['th_des'] is None else s['th_des']
                s['th_des'] = wrap(base + R * (d[5] - d[4]) / B * dt)
        self.push('wl_raw', d[0], 2)
        self.push('wr_raw', d[1], 2)
        self.push('wl_fw', d[2], 2)
        self.push('wr_fw', d[3], 2)

    def on_wheels_k(self, msg):
        if len(msg.data) >= 2:
            self.push('wl_k', msg.data[0], 2)
            self.push('wr_k', msg.data[1], 2)

    def on_imu(self, m, kind):
        gz = m.angular_velocity.z * RAD2DEG
        if kind == 'raw':
            with self.lock:
                self.st['gz'] = gz
        self.push('gz_' + kind, gz, 2)
        self.push('ax_' + kind, m.linear_acceleration.x, 2)

    def on_gimu(self, m, kind):
        self.push('ggy_' + kind, m.angular_velocity.y * RAD2DEG)

    def on_glove_angles(self, msg):
        d = msg.data
        if len(d) >= 4:          # [pitch, roll, pitch_raw, roll_raw]
            self.count['glove'] += 1
            self.t_last['glove'] = time.monotonic()
            self.push('gp_fw', d[0])
            self.push('gr_fw', d[1])
            self.push('gp_raw', d[2])
            self.push('gr_raw', d[3])

    def on_glove_angles_k(self, msg):
        if len(msg.data) >= 2:
            self.push('gp_k', msg.data[0])
            self.push('gr_k', msg.data[1])

    def on_hand(self, msg):
        if len(msg.data) >= 2:
            with self.lock:
                self.st['hand'] = (round(msg.data[0], 1), round(msg.data[1], 1))

    def on_cmd_vel(self, msg):
        with self.lock:
            self.st['cmd_v'], self.st['cmd_w'] = msg.linear.x, msg.angular.z
        self.t_last['cmd'] = time.monotonic()

    def on_goal_state(self, msg):
        try:
            self.goal_state = json.loads(msg.data)
        except ValueError:
            pass

    def send_goal(self, x, y):
        x, y = float(x), float(y)
        if not (math.isfinite(x) and math.isfinite(y)) or abs(x) > 1 or abs(y) > 1:
            return
        self.drive = None
        self.pub_goal.publish(Point(x=x, y=y, z=0.0))
        self.get_logger().info(f'Panel -> objetivo x={x * 100:.1f} cm y={y * 100:.1f} cm')

    def cancel_goal(self):
        self.pub_goal_cancel.publish(Bool(data=True))

    def count_rates(self):
        for k in ('robot', 'glove'):
            self.rate[k] = self.count[k]
            self.count[k] = 0

    # ---------------- salidas ROS ----------------
    def send_cmd(self, txt):
        txt = str(txt).strip()
        if not txt.startswith(ALLOWED_CMDS) or len(txt) > 64:
            return False
        self.pub_cmd.publish(String(data=txt))
        self.get_logger().info(f'Panel -> robot: {txt}')
        return True

    def set_estop(self, on):
        with self.lock:
            self.st['estop'] = bool(on)
        self.pub_estop.publish(Bool(data=bool(on)))
        if on:
            self.drive = None
            self.pub_vel.publish(Twist())
            self.pub_cmd.publish(String(data='S'))

    def set_drive(self, v, w):
        v = max(-0.2, min(0.2, float(v)))
        w = max(-4.0, min(4.0, float(w)))
        self.drive = (v, w, time.monotonic())

    def drive_tick(self):
        if self.drive is None:
            return
        v, w, t = self.drive
        if time.monotonic() - t > 0.4 or self.st['estop']:    # el navegador dejó de enviar
            self.drive = None
            self.pub_vel.publish(Twist())
            return
        m = Twist()
        m.linear.x, m.angular.z = v, w
        self.pub_vel.publish(m)

    def estop_tick(self):
        if self.st['estop']:
            self.pub_estop.publish(Bool(data=True))

    # ---------------- estado para el navegador ----------------
    def snapshot(self, since, want=()):
        now_m = time.monotonic()
        with self.lock:
            s = dict(self.st)
            series = {}
            for k in want:
                d = self.buf.get(k)
                if d is None:
                    continue
                pts = [p for p in d if p[0] > since] if d and d[-1][0] > since else []
                if pts:
                    series[k] = pts
        s['link'] = {k: {'rate': self.rate[k], 'age': round(now_m - self.t_last[k], 2) if self.t_last[k] else None}
                     for k in ('robot', 'glove')}
        s['cmd_age'] = round(now_m - self.t_last['cmd'], 2) if self.t_last['cmd'] else None
        if now_m - self.t_ips > 10:            # la IP puede cambiar si se renueva la red
            self.ips, self.t_ips = lan_ips(), now_m
        s['goal'] = self.goal_state
        s['ips'] = self.ips
        s['port'] = self.get_parameter('port').value
        s['t'] = round(self.now(), 3)
        s['series'] = series
        return s


# ---------------- servidor HTTP + WebSocket (asyncio, biblioteca estándar) ----------------
class PanelServer:
    def __init__(self, node, web_dir, mesh_dir):
        self.node = node
        self.web_dir = os.path.realpath(web_dir)
        self.mesh_dir = os.path.realpath(mesh_dir)

    def resolve(self, path):
        path = path.split('?', 1)[0]
        if path in ('', '/'):
            path = '/index.html'
        if path.startswith('/meshes/'):
            base, rel = self.mesh_dir, path[len('/meshes/'):]
        else:
            base, rel = self.web_dir, path.lstrip('/')
        full = os.path.realpath(os.path.join(base, rel))
        if not full.startswith(base + os.sep) or not os.path.isfile(full):
            return None
        return full

    async def handle(self, reader, writer):
        try:
            head = await asyncio.wait_for(reader.readuntil(b'\r\n\r\n'), 10)
        except Exception:
            writer.close()
            return
        lines = head.decode('latin1').split('\r\n')
        try:
            method, path, _ = lines[0].split(' ', 2)
        except ValueError:
            writer.close()
            return
        hdr = {}
        for ln in lines[1:]:
            if ':' in ln:
                k, v = ln.split(':', 1)
                hdr[k.strip().lower()] = v.strip()
        if hdr.get('upgrade', '').lower() == 'websocket' and path.startswith('/ws'):
            await self.websocket(reader, writer, hdr)
            return
        full = self.resolve(path) if method == 'GET' else None
        if full is None:
            body = b'No encontrado'
            writer.write(b'HTTP/1.1 404 Not Found\r\nContent-Length: %d\r\nConnection: close\r\n\r\n' % len(body) + body)
        else:
            ctype = mimetypes.guess_type(full)[0] or 'application/octet-stream'
            if full.endswith('.js'):
                ctype = 'text/javascript'
            elif full.endswith('.woff2'):
                ctype = 'font/woff2'
            with open(full, 'rb') as fh:
                body = fh.read()
            writer.write((f'HTTP/1.1 200 OK\r\nContent-Type: {ctype}\r\nContent-Length: {len(body)}\r\n'
                          'Cache-Control: no-cache\r\nConnection: close\r\n\r\n').encode() + body)
        try:
            await writer.drain()
        finally:
            writer.close()

    @staticmethod
    def frame(payload, opcode=1):
        n = len(payload)
        if n < 126:
            h = struct.pack('!BB', 0x80 | opcode, n)
        elif n < 65536:
            h = struct.pack('!BBH', 0x80 | opcode, 126, n)
        else:
            h = struct.pack('!BBQ', 0x80 | opcode, 127, n)
        return h + payload

    async def read_frame(self, reader):
        b1, b2 = await reader.readexactly(2)
        opcode, masked, n = b1 & 0x0F, b2 & 0x80, b2 & 0x7F
        if n == 126:
            n = struct.unpack('!H', await reader.readexactly(2))[0]
        elif n == 127:
            n = struct.unpack('!Q', await reader.readexactly(8))[0]
        if n > 1 << 16:
            raise ValueError('mensaje demasiado grande')
        mask = await reader.readexactly(4) if masked else b'\0\0\0\0'
        data = bytearray(await reader.readexactly(n))
        for i in range(n):
            data[i] ^= mask[i % 4]
        return opcode, bytes(data)

    async def websocket(self, reader, writer, hdr):
        key = hdr.get('sec-websocket-key', '')
        acc = base64.b64encode(hashlib.sha1((key + WS_GUID).encode()).digest()).decode()
        writer.write(('HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n'
                      f'Sec-WebSocket-Accept: {acc}\r\n\r\n').encode())
        await writer.drain()
        node = self.node
        period = 1.0 / node.get_parameter('rate').value
        alive = True
        acked = asyncio.Event()
        acked.set()
        want = []                                   # series que pide la vista abierta
        try:
            writer.transport.set_write_buffer_limits(high=8192)
        except Exception:
            pass

        async def sender():
            # Control de flujo: el siguiente estado sale solo cuando el navegador confirmó el
            # anterior (o tras 1 s). Así nunca se acumula una cola y lo que se ve es lo actual,
            # aunque el navegador dibuje lento (p. ej. Firefox sin tarjeta gráfica en la VM).
            nonlocal alive
            since = node.now() - 0.05
            while alive:
                t0 = time.monotonic()
                try:
                    await asyncio.wait_for(acked.wait(), 1.0)
                except asyncio.TimeoutError:
                    pass
                acked.clear()
                snap = node.snapshot(since, want)
                since = snap['t']
                writer.write(self.frame(json.dumps(snap, separators=(',', ':')).encode()))
                try:
                    await writer.drain()
                except Exception:
                    alive = False
                    break
                await asyncio.sleep(max(0.0, period - (time.monotonic() - t0)))

        task = asyncio.ensure_future(sender())
        try:
            while alive:
                opcode, data = await self.read_frame(reader)
                if opcode == 8:
                    break
                if opcode == 9:
                    writer.write(self.frame(data, 10))
                    continue
                if opcode != 1:
                    continue
                try:
                    msg = json.loads(data.decode())
                except ValueError:
                    continue
                t = msg.get('t')
                if t == 'ack':
                    acked.set()
                elif t == 'sub':
                    want[:] = [k for k in msg.get('series', []) if k in node.buf][:24]
                elif t == 'cmd':
                    node.send_cmd(msg.get('v', ''))
                elif t == 'estop':
                    node.set_estop(bool(msg.get('v')))
                elif t == 'goal':
                    node.send_goal(msg.get('x', 0.0), msg.get('y', 0.0))
                elif t == 'goal_cancel':
                    node.cancel_goal()
                elif t == 'drive':
                    node.set_drive(msg.get('v', 0.0), msg.get('w', 0.0))
        except (asyncio.IncompleteReadError, ConnectionError, ValueError):
            pass
        finally:
            alive = False
            task.cancel()
            try:
                writer.write(self.frame(b'', 8))
                writer.close()
            except Exception:
                pass


def main():
    rclpy.init()
    node = PanelNode()
    web_dir = node.get_parameter('web_dir').value
    share = None
    if get_package_share_directory:
        try:
            share = get_package_share_directory('minisumo_ros')
        except Exception:
            share = None
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    web_dir = web_dir or (os.path.join(share, 'web') if share and os.path.isdir(os.path.join(share, 'web'))
                          else os.path.join(here, 'web'))
    mesh_dir = os.path.join(share, 'meshes') if share else os.path.join(here, 'meshes')
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    server = PanelServer(node, web_dir, mesh_dir)
    port = node.get_parameter('port').value

    async def run():
        srv = await asyncio.start_server(server.handle, '0.0.0.0', port)
        node.get_logger().info(f'Panel CAR-LOSS en http://<IP de esta máquina>:{port}  (web: {web_dir})')
        async with srv:
            await srv.serve_forever()

    try:
        asyncio.run(run())
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
