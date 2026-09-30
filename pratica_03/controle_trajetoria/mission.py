"""Exercício 1.2 — seguimento de missão (waypoints).

Segue sequencialmente os waypoints do YAML usando o controlador de pose
escolhido em `mode` (continuous ou three_maneuvers). Um waypoint conta como
atingido quando o robô entra num círculo de raio `waypoint_radius` em volta
dele (a orientação no waypoint não importa).
"""
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.parameter import Parameter

from controle_trajetoria.base_node import ControlNode
from controle_trajetoria.controllers import make_pose_controller


class Mission(ControlNode):

    def __init__(self):
        super().__init__('mission')
        mode = self.param('mode', 'continuous')
        self.controller = make_pose_controller(
            mode,
            linear_gains=self.gains('linear', (0.5, 0.0, 0.0)),
            angular_gains=self.gains('angular', (1.5, 0.0, 0.0)),
            pos_tol=self.param('position_tolerance', 0.05),
            ang_tol=self.param('angle_tolerance', 0.05),
            v_max=self.v_max,
            w_max=self.w_max,
        )

        self.radius = self.param('waypoint_radius', 0.15)

        # Parâmetros do ROS 2 não aceitam lista de listas, então o YAML usa
        # arrays paralelos: o waypoint i é (x[i], y[i])
        for name in ('waypoints_x', 'waypoints_y'):
            self.declare_parameter(name, Parameter.Type.DOUBLE_ARRAY)
        xs = self.get_parameter('waypoints_x').value
        ys = self.get_parameter('waypoints_y').value
        if not len(xs) == len(ys) >= 3:
            raise ValueError('waypoints_x e waypoints_y precisam ter o mesmo '
                             'tamanho (mínimo de 3 waypoints)')
        # theta = 0.0 só preenche o formato do goal: como o waypoint é trocado
        # ao entrar no raio, o controlador nunca chega na fase de orientação
        self.waypoints = [(x, y, 0.0) for x, y in zip(xs, ys)]
        self.index = 0

        self.get_logger().info(f'Missão ({mode}) com {len(self.waypoints)} waypoints')

    def loop(self, dt):
        if self.index >= len(self.waypoints):
            return  # missão concluída

        goal = self.waypoints[self.index]
        distance = math.hypot(goal[0] - self.pose[0], goal[1] - self.pose[1])

        if distance > self.radius:
            v, w, _ = self.controller.compute(self.pose, goal, dt)
            self.publish_cmd(v, w)
        else:
            self.get_logger().info(
                f'Waypoint {self.index + 1}/{len(self.waypoints)} atingido '
                f'({goal[0]:.2f}, {goal[1]:.2f})')
            self.index += 1
            self.controller.reset()
            if self.index == len(self.waypoints):
                self.stop()
                self.get_logger().info('Missão concluída')


def main():
    rclpy.init()
    node = Mission()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()
