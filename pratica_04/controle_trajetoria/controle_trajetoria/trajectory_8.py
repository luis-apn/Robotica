"""Seguimento de trajetória em 8 (prática 03, adaptado na prática 04).

mode = open_loop: publica só as velocidades de referência (v_d, w_d),
                  ignorando a pose medida.
mode = feedback:  realimentação do ponto à frente + feedforward com ganho
                  k_ff (k_ff = 0 → só realimentação).

Prática 04 (exercício 2.1):
  - pose_topic escolhe a realimentação: /odom (rodas) ou /odom_sensores
    (GPS + IMU, com ruído).
  - ground_truth_topic (/ground_truth): pose REAL do Gazebo, usada só para
    avaliar. Ao final o nó loga dois RMSE de posição:
        "medido" → erro em relação à pose que o controlador enxerga
        "real"   → erro em relação à pose verdadeira (ground truth)
    Com ruído alto o controlador pode achar que está seguindo bem enquanto o
    robô real se afasta da trajetória.
  - Também loga a "agitação" dos comandos: RMS da variação de v e w entre
    duas amostras seguidas. Mais ruído nos sensores → comandos mais agitados.
  - anchor_at_start = false: a trajetória é centrada na origem do mundo, onde
    o robô nasce (ancorar na pose medida colocaria o erro do GPS na própria
    referência). true = comportamento da prática 03.

Tópicos para o PlotJuggler:
  /trajetoria_desejada (PoseStamped) — referência
  /ground_truth, /odom, /odom_sensores — pose real, das rodas e dos sensores
  /cmd_vel — sinais de controle
"""
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data

from controle_trajetoria.base_node import ControlNode
from controle_trajetoria.controllers import (
    PointAheadTrajectoryController, figure8, unicycle_feedforward)
from controle_trajetoria.utils import (
    clamp, make_pid, pid_step, quaternion_from_yaw, wrap_angle,
    yaw_from_quaternion)

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
        self.anchor_at_start = self.param('anchor_at_start', True)
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

        # Pose real (só avaliação)
        self.true_pose = None
        gt_topic = self.param('ground_truth_topic', '')
        if gt_topic:
            self.create_subscription(Odometry, gt_topic, self._ground_truth_callback,
                                     qos_profile_sensor_data)

        self.state = ALIGN
        self.t0 = None
        self.origin = None
        self.stats = {'pos': 0.0, 'theta': 0.0, 'pos_real': 0.0, 'dv': 0.0, 'dw': 0.0}
        self.samples = 0
        self.samples_real = 0
        self.last_cmd = None

        self.get_logger().info(
            f'Trajetória em 8 ({self.mode}, k_ff={self.k_ff}, '
            f'pose de {self.pose_topic})')

    def _ground_truth_callback(self, msg):
        p = msg.pose.pose
        self.true_pose = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))

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
        self.origin = self.pose[:2] if self.anchor_at_start else (0.0, 0.0)
        self.controller.reset()
        self.state = RUNNING
        self.get_logger().info(
            f'Iniciando a trajetória (origem = {self.origin[0]:.3f}, {self.origin[1]:.3f})')

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
        v, w = clamp(v, self.v_max), clamp(w, self.w_max)
        self.publish_cmd(v, w)

        # Erro em relação à pose que o controlador enxerga (medida)
        x, y, theta = self.pose
        self.stats['pos'] += math.hypot(x_d - x, y_d - y) ** 2
        self.stats['theta'] += wrap_angle(theta_d - theta) ** 2
        self.samples += 1

        # Erro em relação à pose real (ground truth)
        if self.true_pose is not None:
            xr, yr, _ = self.true_pose
            self.stats['pos_real'] += math.hypot(x_d - xr, y_d - yr) ** 2
            self.samples_real += 1

        # Agitação dos comandos: variação entre duas amostras seguidas
        if self.last_cmd is not None:
            self.stats['dv'] += (v - self.last_cmd[0]) ** 2
            self.stats['dw'] += (w - self.last_cmd[1]) ** 2
        self.last_cmd = (v, w)

        self.publish_reference(x_d, y_d, theta_d)

    def finish(self):
        self.stop()
        self.state = DONE
        n = max(self.samples, 1)
        rmse_pos = math.sqrt(self.stats['pos'] / n)
        rmse_theta = math.sqrt(self.stats['theta'] / n)
        rms_dv = math.sqrt(self.stats['dv'] / max(n - 1, 1))
        rms_dw = math.sqrt(self.stats['dw'] / max(n - 1, 1))
        real = (f'{math.sqrt(self.stats["pos_real"] / self.samples_real):.3f} m'
                if self.samples_real else 'sem ground truth')
        self.get_logger().info(
            f'Fim ({self.mode}, k_ff={self.k_ff}, Ω={self.omega}, pose de {self.pose_topic}): '
            f'RMSE posição medido = {rmse_pos:.3f} m | real = {real} | '
            f'RMSE orientação = {math.degrees(rmse_theta):.1f}° | '
            f'agitação dos comandos: Δv = {rms_dv:.4f} m/s, Δw = {rms_dw:.4f} rad/s '
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
