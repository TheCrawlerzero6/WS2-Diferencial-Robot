"""Ejecuta el cuatro y las pruebas básicas a partir de la odometría."""

import math
import time

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

from ws2_trajectory.control import Controller, four_points, Settings, transform_points, Watchdog


class Trayectoria4(Node):
    """Publica órdenes acotadas y detiene el robot cuando termina o faltan datos."""

    def __init__(self, diagnostic=False):
        """Configura una sola rutina; requiere reiniciar el nodo para repetirla."""
        super().__init__('prueba_movimiento' if diagnostic else 'trayectoria_4')
        defaults = {
            'velocidad_lineal': 0.12, 'velocidad_angular': 0.6,
            'tolerancia_posicion': 0.02, 'tolerancia_angular': 0.03,
            'escala': 1.0, 'distancia': 0.5, 'angulo': math.pi / 2,
            'espera_odom': 10.0, 'timeout_odom': 5.0, 'tiempo_maximo': 180.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.mode = self.declare_parameter('modo', 'recto' if diagnostic else 'cuatro').value
        if self.mode not in ('cuatro', 'recto', 'giro', 'parada'):
            raise ValueError('Modo válido: cuatro, recto, giro o parada.')
        self.values = {name: float(self.get_parameter(name).value) for name in defaults}
        if not all(math.isfinite(value) for value in self.values.values()):
            raise ValueError('Los parámetros numéricos deben ser finitos.')
        for name in ('espera_odom', 'timeout_odom', 'tiempo_maximo', 'distancia'):
            if self.values[name] <= 0:
                raise ValueError(f'{name} debe ser positivo.')
        self.settings = Settings(
            linear_speed=self.values['velocidad_lineal'],
            angular_speed=self.values['velocidad_angular'],
            position_tolerance=self.values['tolerancia_posicion'],
            angle_tolerance=self.values['tolerancia_angular'],
        )
        four_points(self.values['escala'])
        self.velocity_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(Path, '/recorrido', retained)
        self.target_pub = self.create_publisher(Path, '/recorrido_objetivo', retained)
        self.status_pub = self.create_publisher(String, '/estado_recorrido', retained)
        self.controller = None
        self.pose = None
        self.failed = False
        self.finished = False
        self.path = Path()
        self.path.header.frame_id = 'odom'
        self.last_path_stamp = None
        self.watchdog = Watchdog(
            time.monotonic(), self.values['espera_odom'],
            self.values['timeout_odom'], self.values['tiempo_maximo'],
        )
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        # El reloj estable sigue funcionando cuando Gazebo deja de publicar /clock.
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1.0 / 30.0, self.update, clock=self.wall_clock)
        self.status('esperando_odometria')
        self.get_logger().info(f'Modo {self.mode}: esperando odometría en /odom.')

    def status(self, value):
        """Publica el estado para la terminal y las pruebas de integración."""
        message = String()
        message.data = value
        self.status_pub.publish(message)

    def fail(self, reason):
        """Enclava la parada para impedir un reinicio inesperado del movimiento."""
        if not self.failed:
            self.failed = True
            self.velocity_pub.publish(Twist())
            self.status('error: ' + reason)
            self.get_logger().error(reason)

    def on_odom(self, message):
        """Valida y registra la pose; fija los objetivos respecto al inicio."""
        if self.failed:
            return
        p, q = message.pose.pose.position, message.pose.pose.orientation
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        norm = math.sqrt(sum(value * value for value in values[3:]))
        if not all(math.isfinite(value) for value in values) or norm < 1e-6:
            self.fail('La odometría contiene una pose inválida.')
            return
        if message.header.frame_id != 'odom' or message.child_frame_id != 'base_footprint':
            self.fail('Se esperaba odometría de odom a base_footprint.')
            return
        qx, qy, qz, qw = (value / norm for value in values[3:])
        yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        self.pose = (p.x, p.y, yaw)
        sim_time = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        try:
            self.watchdog.observe(time.monotonic(), sim_time)
        except ValueError as error:
            self.fail(str(error))
            return
        if self.controller is None:
            points = []
            final_yaw = None
            if self.mode == 'cuatro':
                points = four_points(self.values['escala'])
            elif self.mode == 'recto':
                points = [(self.values['distancia'], 0.0)]
            elif self.mode == 'giro':
                final_yaw = yaw + self.values['angulo']
            targets = transform_points(points, self.pose)
            self.controller = Controller(targets, self.settings, final_yaw)
            target_path = Path()
            target_path.header = message.header
            for x, y in [(p.x, p.y)] + targets:
                target = PoseStamped()
                target.header = message.header
                target.pose.position.x = x
                target.pose.position.y = y
                target.pose.orientation.w = 1.0
                target_path.poses.append(target)
            self.target_pub.publish(target_path)
            self.status('ejecutando')
            self.get_logger().info(
                f'Inicio: x={p.x:.3f}, y={p.y:.3f}, yaw={yaw:.3f} rad.'
            )
        if self.last_path_stamp is None or sim_time - self.last_path_stamp >= 0.1:
            point = PoseStamped()
            point.header = message.header
            point.pose = message.pose.pose
            self.path.header = message.header
            self.path.poses.append(point)
            self.path.poses = self.path.poses[-4000:]
            self.path_pub.publish(self.path)
            self.last_path_stamp = sim_time

    def update(self):
        """Publica a 30 Hz; la orden de parada se mantiene mientras el nodo vive."""
        command = Twist()
        if not self.failed and not self.finished:
            reason = self.watchdog.failure(time.monotonic())
            if reason:
                self.fail(reason)
            elif self.controller is not None:
                command.linear.x, command.angular.z = self.controller.step(
                    self.pose, self.watchdog.last_sim
                )
                if self.controller.done:
                    self.finished = True
                    self.status('finalizado')
                    self.get_logger().info('Recorrido finalizado. Robot detenido.')
        self.velocity_pub.publish(command)

    def stop(self):
        """Envía varias órdenes nulas antes de cerrar la comunicación."""
        for _ in range(10):
            if not rclpy.ok():
                break
            self.velocity_pub.publish(Twist())
            time.sleep(0.02)


def run(args=None, diagnostic=False):
    """Mantiene ROS activo durante la parada por Ctrl+C."""
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = Trayectoria4(diagnostic)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None):
    """Ejecuta el número cuatro."""
    run(args)


def prueba_main(args=None):
    """Ejecuta una comprobación de avance, giro o parada."""
    run(args, diagnostic=True)
