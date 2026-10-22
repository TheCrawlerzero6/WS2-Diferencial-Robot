"""Comprueba el recorrido del cuatro y la parada del controlador."""

import math

import pytest

from ws2_trajectory.control import Controller, four_points
from ws2_trajectory.control import wrap_angle


def simulate(controller, initial=(0.0, 0.0, 0.0), duration=160.0):
    """Integra órdenes y conserva el historial de puntos alcanzados."""
    x, y, yaw = initial
    history = []
    for step in range(int(duration * 30)):
        v, w = controller.step((x, y, yaw), step / 30)
        assert abs(v) <= controller.settings.linear_speed
        assert abs(w) <= controller.settings.angular_speed
        x += v * math.cos(yaw) / 30
        y += v * math.sin(yaw) / 30
        yaw = wrap_angle(yaw + w / 30)
        history.append((x, y, yaw, controller.index))
        if controller.done:
            return (x, y, yaw), history
    pytest.fail('El controlador no terminó dentro del tiempo previsto.')


def test_four():
    """Comprueba que el controlador alcance los cuatro puntos del recorrido."""
    goals = four_points()
    controller = Controller(goals)
    final, history = simulate(controller)
    for index, goal in enumerate(goals):
        distances = [math.hypot(x - goal[0], y - goal[1])
                     for x, y, _, current in history if current == index]
        assert min(distances) <= 0.02
    assert math.hypot(final[0] - goals[-1][0], final[1] - goals[-1][1]) < 0.02
    assert controller.done


def test_stop():
    """Comprueba que las órdenes permanezcan nulas después de terminar."""
    controller = Controller([(0.5, 0.0)])
    final, _ = simulate(controller)
    assert controller.done
    for now in (20.0, 21.0, 22.0):
        assert controller.step(final, now) == (0.0, 0.0)
