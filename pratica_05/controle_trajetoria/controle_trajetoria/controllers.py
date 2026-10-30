"""Leis de controle da prática 03 (Python puro, sem ROS).

Convenção: pose = (x, y, theta) e goal = (x_d, y_d, theta_d). Os métodos
compute() dos controladores de pose retornam (v, w, done).
"""
import math

from controle_trajetoria.utils import make_pid, pid_step, wrap_angle


def polar_error(pose, goal):
    """Retorna (rho, alpha): distância até o objetivo e desalinhamento."""
    x, y, theta = pose
    dx = goal[0] - x
    dy = goal[1] - y
    rho = math.hypot(dx, dy)
    alpha = wrap_angle(math.atan2(dy, dx) - theta)
    return rho, alpha


class ContinuousPoseController:
    """Controle de pose contínuo: v e w comandados ao mesmo tempo.

    Fase 1 (posição):    v = PID_v(rho) * cos(alpha),  w = PID_w(alpha)
    Fase 2 (orientação): v = 0,                        w = PID_w(theta_d - theta)
    """

    def __init__(self, linear_gains, angular_gains, pos_tol, ang_tol,
                 v_max, w_max):
        self.pid_v = make_pid(linear_gains, v_max)
        self.pid_w = make_pid(angular_gains, w_max)
        self.pos_tol = pos_tol
        self.ang_tol = ang_tol
        self.reset()

    def reset(self):
        """Zera o estado interno (chamar ao trocar de objetivo)."""
        self.pid_v.reset()
        self.pid_w.reset()
        self.position_reached = False

    def compute(self, pose, goal, dt):
        rho, alpha = polar_error(pose, goal)

        # Trava a fase 2: perto do alvo o alpha fica instável (atan2 de um
        # vetor minúsculo), então depois de chegar não volta pra fase 1.
        if not self.position_reached and rho < self.pos_tol:
            self.position_reached = True
            self.pid_w.reset()

        if not self.position_reached:
            # cos(alpha) reduz v quando o robô não aponta pro alvo e o faz
            # andar de ré se o alvo estiver atrás (cos < 0)
            v = pid_step(self.pid_v, rho, dt) * math.cos(alpha)
            w = pid_step(self.pid_w, alpha, dt)
            return v, w, False

        e_theta = wrap_angle(goal[2] - pose[2])
        if abs(e_theta) < self.ang_tol:
            return 0.0, 0.0, True
        return 0.0, pid_step(self.pid_w, e_theta, dt), False


class ThreeManeuverPoseController:
    """Controle de pose com três manobras: rotação, translação, rotação.

    ROTATE_1:  gira no lugar até apontar pro alvo           (v = 0)
    TRANSLATE: anda reto, com pequena correção de rumo
    ROTATE_2:  gira no lugar até a orientação final         (v = 0)
    """

    ROTATE_1 = 'ROTATE_1'
    TRANSLATE = 'TRANSLATE'
    ROTATE_2 = 'ROTATE_2'
    DONE = 'DONE'

    # Se durante a translação o desalinhamento passar disso, volta a girar
    REALIGN_ANGLE = math.pi / 4

    def __init__(self, linear_gains, angular_gains, pos_tol, ang_tol,
                 v_max, w_max):
        self.pid_v = make_pid(linear_gains, v_max)
        self.pid_w = make_pid(angular_gains, w_max)
        self.pos_tol = pos_tol
        self.ang_tol = ang_tol
        self.reset()

    def reset(self):
        """Zera o estado interno (chamar ao trocar de objetivo)."""
        self._set_state(self.ROTATE_1)

    def _set_state(self, state):
        self.state = state
        self.pid_v.reset()
        self.pid_w.reset()

    def compute(self, pose, goal, dt):
        rho, alpha = polar_error(pose, goal)

        if self.state == self.ROTATE_1:
            if rho < self.pos_tol:
                self._set_state(self.ROTATE_2)
            elif abs(alpha) < self.ang_tol:
                self._set_state(self.TRANSLATE)
            else:
                return 0.0, pid_step(self.pid_w, alpha, dt), False

        if self.state == self.TRANSLATE:
            if rho < self.pos_tol:
                self._set_state(self.ROTATE_2)
            elif abs(alpha) > self.REALIGN_ANGLE:
                # Desviou demais (ou passou do alvo): gira de novo
                self._set_state(self.ROTATE_1)
                return 0.0, 0.0, False
            else:
                # Distância projetada na direção do robô: fica negativa se
                # ele passar do alvo, o que faz o PID dar ré
                x, y, theta = pose
                along = ((goal[0] - x) * math.cos(theta)
                         + (goal[1] - y) * math.sin(theta))
                v = pid_step(self.pid_v, along, dt)
                w = pid_step(self.pid_w, alpha, dt)
                return v, w, False

        if self.state == self.ROTATE_2:
            e_theta = wrap_angle(goal[2] - pose[2])
            if abs(e_theta) < self.ang_tol:
                self._set_state(self.DONE)
            else:
                return 0.0, pid_step(self.pid_w, e_theta, dt), False

        return 0.0, 0.0, True


def make_pose_controller(mode, linear_gains, angular_gains, pos_tol, ang_tol,
                         v_max, w_max):
    """Cria o controlador de pose pelo nome do modo."""
    classes = {
        'continuous': ContinuousPoseController,
        'three_maneuvers': ThreeManeuverPoseController,
    }
    if mode not in classes:
        raise ValueError(f"mode deve ser um de {list(classes)}, veio '{mode}'")
    return classes[mode](linear_gains, angular_gains, pos_tol, ang_tol,
                         v_max, w_max)


# ---------------------------------------------------------------------------
# Controle de trajetória
# ---------------------------------------------------------------------------

def figure8(t, A, B, omega):
    """Trajetória em 8 (lemniscata de Gerono) e suas derivadas.

    x_d = A sin(Ωt)
    y_d = B sin(Ωt) cos(Ωt) = (B/2) sin(2Ωt)

    Retorna (x, y, dx, dy, ddx, ddy).
    """
    s, c = math.sin(omega * t), math.cos(omega * t)
    s2, c2 = math.sin(2 * omega * t), math.cos(2 * omega * t)
    x = A * s
    y = 0.5 * B * s2
    dx = A * omega * c
    dy = B * omega * c2
    ddx = -A * omega ** 2 * s
    ddy = -2.0 * B * omega ** 2 * s2
    return x, y, dx, dy, ddx, ddy


def unicycle_feedforward(dx, dy, ddx, ddy):
    """Velocidades (v_d, w_d) e rumo theta_d que um uniciclo precisa ter.

    v_d = sqrt(dx² + dy²)
    w_d = (dx·ddy − dy·ddx) / (dx² + dy²)      (curvatura × velocidade)
    """
    speed_sq = dx * dx + dy * dy
    v_d = math.sqrt(speed_sq)
    w_d = (dx * ddy - dy * ddx) / speed_sq if speed_sq > 1e-9 else 0.0
    theta_d = math.atan2(dy, dx)
    return v_d, w_d, theta_d


class PointAheadTrajectoryController:
    """Seguimento de trajetória com realimentação + feedforward.

    Controla um ponto P a uma distância d à frente do eixo das rodas, que
    (ao contrário do eixo) pode se mover em qualquer direção:

        P     = [x + d cosθ,  y + d sinθ]
        u     = K_ff · Ṗ_ref + PID(P_ref − P)          (um PID por eixo)
        v     =  cosθ · u_x + sinθ · u_y
        w     = (−sinθ · u_x + cosθ · u_y) / d

    P_ref é o ponto à frente da *pose desejada*, então quando P segue P_ref
    o centro do robô segue exatamente a trajetória (sem atraso de d).
    """

    def __init__(self, gains, k_ff, d):
        # Sem limite no PID: a saturação é feita em v e w no nó
        self.pid_x = make_pid(gains, math.inf)
        self.pid_y = make_pid(gains, math.inf)
        self.k_ff = k_ff
        self.d = d

    def reset(self):
        self.pid_x.reset()
        self.pid_y.reset()

    def compute(self, pose, ref, dt):
        """Retorna (v, w). ref = (x_d, y_d, theta_d, v_d, w_d)."""
        x, y, theta = pose
        x_d, y_d, theta_d, v_d, w_d = ref
        d = self.d

        # Ponto à frente de referência e sua velocidade
        px_ref = x_d + d * math.cos(theta_d)
        py_ref = y_d + d * math.sin(theta_d)
        vx_ref = v_d * math.cos(theta_d) - d * w_d * math.sin(theta_d)
        vy_ref = v_d * math.sin(theta_d) + d * w_d * math.cos(theta_d)

        # Ponto à frente do robô
        px = x + d * math.cos(theta)
        py = y + d * math.sin(theta)

        ux = self.k_ff * vx_ref + pid_step(self.pid_x, px_ref - px, dt)
        uy = self.k_ff * vy_ref + pid_step(self.pid_y, py_ref - py, dt)

        v = math.cos(theta) * ux + math.sin(theta) * uy
        w = (-math.sin(theta) * ux + math.cos(theta) * uy) / d
        return v, w
