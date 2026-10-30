"""Exercício 1.1 — controle de pose.

Lê a pose desejada do tópico /pose_desejada (geometry_msgs/Pose2D) e publica
em /cmd_vel. O parâmetro `mode` escolhe a lei de controle:
  - continuous:      v e w ao mesmo tempo
  - three_maneuvers: rotação → translação → rotação (bônus)

Teste:
  ros2 topic pub --once /pose_desejada geometry_msgs/msg/Pose2D \
    "{x: 2.0, y: 1.0, theta: 1.57}"
"""
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Pose2D

from controle_trajetoria.base_node import ControlNode
from controle_trajetoria.controllers import make_pose_controller


class PoseController(ControlNode):

    def __init__(self):
        super().__init__('pose_controller')
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
        self.goal = None
        self.create_subscription(Pose2D, '/pose_desejada', self.goal_callback, 10)
        self.get_logger().info(f'Controle de pose ({mode}) aguardando /pose_desejada')

    def goal_callback(self, msg):
        self.goal = (msg.x, msg.y, msg.theta)
        self.controller.reset()
        self.get_logger().info(
            f'Novo objetivo: x={msg.x:.2f} y={msg.y:.2f} '
            f'θ={math.degrees(msg.theta):.1f}°')

    def loop(self, dt):
        if self.goal is None:
            return  # sem objetivo: não publica nada (deixa o teleop livre)

        v, w, done = self.controller.compute(self.pose, self.goal, dt)
        self.publish_cmd(v, w)

        if done:
            x, y, theta = self.pose
            self.get_logger().info(
                f'Pose atingida: x={x:.3f} y={y:.3f} θ={math.degrees(theta):.1f}°')
            self.goal = None


def main():
    rclpy.init()
    node = PoseController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()
