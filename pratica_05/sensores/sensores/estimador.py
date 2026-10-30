"""Estimador de pose (x, y, θ) a partir do GPS e da IMU (Python puro, sem ROS).

    θ:      integração da velocidade angular do giroscópio (IMU):
                θ_k = θ_{k-1} + ω_z · dt
            O valor inicial vem da orientação da IMU (alinhamento inicial,
            como uma bússola). Com yaw_source = 'orientation', usa direto a
            orientação da IMU a cada mensagem.

    x, y:   posição do GPS (já em metros, no plano local), corrigida pelo
            braço de alavanca: a antena não fica no centro do robô, então
                p_base = p_gps − R(θ) · offset_gps

Não há filtro: o ruído dos sensores passa direto para a pose. É o "estado
bruto" que entra no filtro de Kalman e aparece no plot do exercício 2.1.
"""
import math


def wrap_angle(a):
    """Normaliza o ângulo para [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


class GpsImuEstimator:

    def __init__(self, gps_offset=(0.0, 0.0), yaw_source='gyro'):
        if yaw_source not in ('gyro', 'orientation'):
            raise ValueError("yaw_source deve ser 'gyro' ou 'orientation'")
        self.gps_offset = gps_offset
        self.yaw_source = yaw_source
        self.theta = None      # θ estimado
        self.gps_xy = None     # última posição da antena (m)
        self.last_imu_t = None

    def update_imu(self, t, wz, yaw_orientation):
        """Nova leitura da IMU no instante t (s)."""
        if self.theta is None or self.yaw_source == 'orientation':
            self.theta = yaw_orientation
        else:
            dt = t - self.last_imu_t
            if dt > 0.0:
                self.theta = wrap_angle(self.theta + wz * dt)
        self.last_imu_t = t

    def update_gps(self, x, y):
        """Nova posição da antena do GPS (m, plano local)."""
        self.gps_xy = (x, y)

    def ready(self):
        return self.theta is not None and self.gps_xy is not None

    def pose(self):
        """Pose estimada (x, y, θ) do centro do robô (base_link)."""
        ox, oy = self.gps_offset
        c, s = math.cos(self.theta), math.sin(self.theta)
        x = self.gps_xy[0] - (c * ox - s * oy)
        y = self.gps_xy[1] - (s * ox + c * oy)
        return x, y, self.theta
