"""Base comum dos nós de controle: odometria, /cmd_vel e parâmetros."""
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time

from controle_trajetoria.utils import clamp, yaw_from_quaternion


class ControlNode(Node):
    """Nó que roda loop(dt) a cada /odom recebido e publica em /cmd_vel.

    O laço roda no callback do /odom (50 Hz) em vez de num timer: com
    use_sim_time o timer só dispara quando o /clock avança, e o Gazebo
    publica o /clock a apenas 10 Hz. O dt vem do carimbo de tempo da
    própria odometria (tempo simulado).
    """

    def __init__(self, name):
        super().__init__(name)
        self.v_max = self.param('v_max', 0.5)
        self.w_max = self.param('w_max', 1.5)

        self.pose = None   # (x, y, theta) da última odometria
        self.stamp = None  # tempo (s) da última odometria

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self._odom_callback, 10)

    def param(self, name, default):
        """Declara um parâmetro (com valor padrão) e retorna seu valor."""
        self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def gains(self, prefix, default=(0.0, 0.0, 0.0)):
        """Lê os ganhos <prefix>.kp, <prefix>.ki, <prefix>.kd do YAML."""
        return tuple(self.param(f'{prefix}.{k}', d)
                     for k, d in zip(('kp', 'ki', 'kd'), default))

    def _odom_callback(self, msg):
        stamp = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        dt = stamp - self.stamp if self.stamp is not None else 0.0

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        theta = yaw_from_quaternion(msg.pose.pose.orientation)
        self.pose = (x, y, theta)
        self.stamp = stamp

        if dt > 0.0:  # ignora a primeira mensagem e mensagens repetidas
            self.loop(dt)

    def loop(self, dt):
        raise NotImplementedError

    def publish_cmd(self, v, w):
        """Publica (v, w) no /cmd_vel, saturando em v_max e w_max."""
        msg = Twist()
        msg.linear.x = clamp(v, self.v_max)
        msg.angular.z = clamp(w, self.w_max)
        self.cmd_pub.publish(msg)

    def stop(self):
        self.publish_cmd(0.0, 0.0)
