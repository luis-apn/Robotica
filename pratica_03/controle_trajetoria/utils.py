import math

from simple_pid.PID import PID


def yaw_from_quaternion(q):
    """Retorna o yaw (rad) de um geometry_msgs/Quaternion."""
    # sin(θ) e cos(θ) extraídos do quatérnio (pitch = roll = 0 no plano)
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(theta):
    """Retorna (x, y, z, w) de uma rotação theta em torno de Z."""
    return 0.0, 0.0, math.sin(theta / 2.0), math.cos(theta / 2.0)


def wrap_angle(a):
    """Normaliza o ângulo para [-pi, pi]."""
    # sin/cos "esquecem" voltas completas; atan2 devolve em [-pi, pi]
    return math.atan2(math.sin(a), math.cos(a))


def clamp(value, limit):
    """Satura value em [-limit, limit]."""
    return max(-limit, min(limit, value))


def make_pid(gains, limit):
    """Cria um PID do simple_pid, com gains = (kp, ki, kd), sobre o erro.

    Convenção usada no pacote: setpoint = 0 e entrada = -erro, pois o
    simple_pid calcula erro = setpoint - entrada. Assim pid(-e) = Kp*e + ...
    """
    # sample_time=None: o passo é dado pelo nosso timer (dt de tempo simulado)
    kp, ki, kd = gains
    return PID(kp, ki, kd, setpoint=0.0, sample_time=None,
               output_limits=(-limit, limit))


def pid_step(pid, error, dt):
    """Executa um passo do PID sobre o erro e retorna a saída."""
    return pid(-error, dt=dt)
