"""Control planar por puntos, independiente de ROS para poder probarlo."""

from dataclasses import dataclass
import math


def wrap_angle(angle):
    """Devuelve un ángulo en el intervalo [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class Settings:
    """Límites y tolerancias del control."""

    linear_speed: float = 0.12
    angular_speed: float = 0.6
    position_tolerance: float = 0.02
    angle_tolerance: float = 0.03
    settle_time: float = 0.25

    def __post_init__(self):
        """Rechaza valores que impedirían completar el movimiento."""
        for value in vars(self).values():
            if not math.isfinite(value) or value <= 0:
                raise ValueError('Los límites y tolerancias deben ser positivos y finitos.')
        if self.linear_speed > 0.2 or self.angular_speed > 0.8:
            raise ValueError('La velocidad excede los límites del modelo (0.2 m/s, 0.8 rad/s).')


def four_points(scale=1.0):
    """Define el 4 con diagonal, de un metro de ancho por dos de alto."""
    if not math.isfinite(scale) or scale < 0.25:
        raise ValueError('La escala debe ser finita y mayor o igual que 0.25.')
    return [(scale * x, scale * y) for x, y in [(-1, -1), (0, -1), (0, 0), (0, -2)]]


def transform_points(points, origin):
    """Lleva puntos relativos al sistema de odometría inicial."""
    ox, oy, yaw = origin
    c, s = math.cos(yaw), math.sin(yaw)
    return [(ox + c * x - s * y, oy + s * x + c * y) for x, y in points]


class Controller:
    """Alterna giros, avance y asentamiento hasta completar los puntos."""

    def __init__(self, points, settings=None, final_yaw=None):
        """Recibe objetivos expresados en el mismo marco que la pose."""
        self.points = list(points)
        self.settings = settings or Settings()
        self.final_yaw = final_yaw
        self.index = 0
        self.state = 'align'
        self.settle_until = None
        self.done = False

    def step(self, pose, now):
        """Calcula velocidad lineal y angular; al terminar devuelve ceros."""
        if not all(math.isfinite(value) for value in (*pose, now)):
            raise ValueError('La pose y el tiempo deben ser finitos.')
        if self.done:
            return 0.0, 0.0
        if self.settle_until is not None:
            if now < self.settle_until:
                return 0.0, 0.0
            self.settle_until = None
            self.index += 1
            self.state = 'align'
        x, y, yaw = pose
        cfg = self.settings
        if self.index >= len(self.points):
            error = 0.0 if self.final_yaw is None else wrap_angle(self.final_yaw - yaw)
            if abs(error) <= cfg.angle_tolerance:
                self.done = True
                return 0.0, 0.0
            return 0.0, self._turn(error)
        gx, gy = self.points[self.index]
        distance = math.hypot(gx - x, gy - y)
        if distance <= cfg.position_tolerance:
            self.settle_until = now + cfg.settle_time
            return 0.0, 0.0
        error = wrap_angle(math.atan2(gy - y, gx - x) - yaw)
        if self.state == 'align':
            if abs(error) > cfg.angle_tolerance:
                return 0.0, self._turn(error)
            self.state = 'drive'
        if abs(error) > 0.35:
            self.state = 'align'
            return 0.0, self._turn(error)
        linear = min(cfg.linear_speed, 1.5 * distance) * math.cos(error)
        return linear, self._turn(error)

    def _turn(self, error):
        return max(-self.settings.angular_speed, min(self.settings.angular_speed, 2.5 * error))


class Watchdog:
    """Vigila recepción de odometría y duración usando relojes separados."""

    def __init__(self, wall_now, startup_timeout=10.0, odom_timeout=5.0, max_duration=180.0):
        """Usa tiempo real para detectar ausencia de datos incluso en pausa."""
        self.created = wall_now
        self.startup_timeout = startup_timeout
        self.odom_timeout = odom_timeout
        self.max_duration = max_duration
        self.last_wall = None
        self.first_sim = None
        self.last_sim = None

    def observe(self, wall_now, sim_now):
        """Actualiza la recepción y rechaza reinicios del reloj durante una ejecución."""
        if self.last_sim is not None and sim_now < self.last_sim:
            raise ValueError('El reloj de simulación retrocedió; reinicia el recorrido.')
        self.last_wall = wall_now
        self.last_sim = sim_now
        if self.first_sim is None:
            self.first_sim = sim_now

    def failure(self, wall_now):
        """Devuelve el motivo de parada o None si los datos siguen vigentes."""
        if self.last_wall is None:
            if wall_now - self.created > self.startup_timeout:
                return 'No llegó odometría durante el tiempo de espera.'
        elif wall_now - self.last_wall > self.odom_timeout:
            return 'Se perdió la odometría o se pausó la simulación.'
        elif self.last_sim - self.first_sim > self.max_duration:
            return 'El movimiento superó el tiempo máximo.'
        return None
