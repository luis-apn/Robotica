"""Exercícios 2.1 e 2.2 — seguimento de trajetória em 8.

mode = open_loop: publica só as velocidades de referência (v_d, w_d),
                  ignorando a odometria.
mode = feedback:  realimentação do ponto à frente + feedforward com ganho
                  k_ff (k_ff = 0 → só realimentação).

A trajetória é ancorada na posição do robô no início (então dá pra rodar de
novo sem reiniciar o Gazebo). Com align_first=true o robô primeiro gira no
lugar até o rumo inicial da trajetória, para a comparação ser justa.

Tópicos para o PlotJuggler:
  /trajetoria_desejada (PoseStamped) — referência
  /odom                              — trajetória real
Ao final de `laps` voltas, o RMSE de posição e de orientação vai para o log.
"""
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseStamped

from controle_trajetoria.base_node import ControlNode
from controle_trajetoria.controllers import (
    PointAheadTrajectoryController, figure8, unicycle_feedforward)
from controle_trajetoria.utils import (
    make_pid, pid_step, quaternion_from_yaw, wrap_angle)

ALIGN, RUNNING, DONE = 'ALIGN', 'RUNNING', 'DONE'


class Trajectory8(ControlNode):

    def __init__(self):
        super().__init__('trajectory_8')
        self.A = self.param('A', 1.0)
        self.B = self.param('B', 2.0)
        self.omega = self.param('Omega', 0.5)
        self.laps = self.param('laps', 1.0)
        self.mode = self.param('mode', 'feedback')
        self.k_ff = self.param('k_ff', 1.0)
        if self.mode not in ('open_loop', 'feedback'):
            raise ValueError("mode deve ser 'open_loop' ou 'feedback'")

        self.controller = PointAheadTrajectoryController(
            gains=self.gains('feedback', (1.0, 0.0, 0.0)),
            k_ff=self.k_ff,
            d=self.param('d', 0.2),
        )

        self.align_first = self.param('align_first', True)
        self.align_pid = make_pid(self.gains('align', (1.0, 0.0, 0.0)), self.w_max)
        self.align_tol = self.param('align_tolerance', 0.05)

        self.ref_pub = self.create_publisher(PoseStamped, '/trajetoria_desejada', 10)

        self.state = ALIGN
        self.t0 = None
        self.origin = None
        self.sq_pos = 0.0
        self.sq_theta = 0.0
        self.samples = 0

        self.get_logger().info(f'Trajetória em 8 ({self.mode}, k_ff={self.k_ff})')

    def reference(self, t):
        """Retorna (x_d, y_d, theta_d, v_d, w_d) no instante t."""
        x, y, dx, dy, ddx, ddy = figure8(t, self.A, self.B, self.omega)
        v_d, w_d, theta_d = unicycle_feedforward(dx, dy, ddx, ddy)
        return self.origin[0] + x, self.origin[1] + y, theta_d, v_d, w_d

    def loop(self, dt):
        if self.state == ALIGN:
            self.align(dt)
        elif self.state == RUNNING:
            self.run(dt)

    def align(self, dt):
        """Gira no lugar até o rumo inicial da trajetória, depois começa."""
        _, _, theta_start = unicycle_feedforward(*figure8(0.0, self.A, self.B, self.omega)[2:])
        error = wrap_angle(theta_start - self.pose[2])
        if self.align_first and abs(error) > self.align_tol:
            self.publish_cmd(0.0, pid_step(self.align_pid, error, dt))
            return

        self.t0 = self.stamp
        self.origin = self.pose[:2]
        self.controller.reset()
        self.state = RUNNING
        self.get_logger().info('Iniciando a trajetória')

    def run(self, dt):
        t = self.stamp - self.t0
        if t >= self.laps * 2.0 * math.pi / self.omega:
            self.finish()
            return

        ref = self.reference(t)
        x_d, y_d, theta_d, v_d, w_d = ref

        if self.mode == 'open_loop':
            v, w = v_d, w_d
        else:
            v, w = self.controller.compute(self.pose, ref, dt)
        self.publish_cmd(v, w)

        # Erros para o RMSE (bônus do 2.2)
        x, y, theta = self.pose
        e_pos = math.hypot(x_d - x, y_d - y)
        e_theta = wrap_angle(theta_d - theta)
        self.sq_pos += e_pos ** 2
        self.sq_theta += e_theta ** 2
        self.samples += 1

        self.publish_reference(x_d, y_d, theta_d)

    def finish(self):
        self.stop()
        self.state = DONE
        rmse_pos = math.sqrt(self.sq_pos / max(self.samples, 1))
        rmse_theta = math.sqrt(self.sq_theta / max(self.samples, 1))
        self.get_logger().info(
            f'Fim ({self.mode}, k_ff={self.k_ff}, Ω={self.omega}): '
            f'RMSE posição = {rmse_pos:.3f} m, '
            f'RMSE orientação = {math.degrees(rmse_theta):.1f}° '
            f'({self.samples} amostras)')

    def publish_reference(self, x, y, theta):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.pose.position.x = x
        msg.pose.position.y = y
        q = msg.pose.orientation
        q.x, q.y, q.z, q.w = quaternion_from_yaw(theta)
        self.ref_pub.publish(msg)


def main():
    rclpy.init()
    node = Trajectory8()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()
