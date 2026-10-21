"""Exercício 1.1 — conversão do GPS (lat, lon, alt) para (X, Y, Z).

Assina:  /gps/fix  (sensor_msgs/NavSatFix)
Publica: /gps/xyz  (geometry_msgs/PointStamped, frame 'map', em metros)

A origem do plano local é dada pelos parâmetros origin_lat/lon/alt, que devem
ser os mesmos do <spherical_coordinates> do mundo (worlds/sensores.world).
Assim (0, 0, 0) é a origem do mundo do Gazebo, onde o robô nasce, e o GPS
convertido pode ser comparado direto com o /odom e o /ground_truth.
Com use_first_fix = true, a origem passa a ser a primeira leitura do GPS.
"""
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, NavSatStatus

from sensores.geodesia import LocalFrame


class GpsToXYZ(Node):

    def __init__(self):
        super().__init__('gps_to_xyz')
        self.declare_parameter('origin_lat', -7.1395)
        self.declare_parameter('origin_lon', -34.8461)
        self.declare_parameter('origin_alt', 40.0)
        self.declare_parameter('use_first_fix', False)
        self.declare_parameter('frame_id', 'map')

        self.frame_id = self.get_parameter('frame_id').value
        self.frame = None
        if not self.get_parameter('use_first_fix').value:
            self.frame = LocalFrame(self.get_parameter('origin_lat').value,
                                    self.get_parameter('origin_lon').value,
                                    self.get_parameter('origin_alt').value)

        # Plugins de sensor do Gazebo publicam com QoS "sensor data"
        # (best effort): o assinante precisa ser compatível
        self.create_subscription(NavSatFix, '/gps/fix', self.fix_callback,
                                 qos_profile_sensor_data)
        self.pub = self.create_publisher(PointStamped, '/gps/xyz', 10)
        self.get_logger().info('Convertendo /gps/fix → /gps/xyz')

    def fix_callback(self, msg):
        if msg.status.status < NavSatStatus.STATUS_FIX:
            return  # sem fix: medição inválida

        if self.frame is None:
            self.frame = LocalFrame(msg.latitude, msg.longitude, msg.altitude)
            self.get_logger().info(
                f'Origem = primeiro fix: lat={msg.latitude:.7f} '
                f'lon={msg.longitude:.7f} alt={msg.altitude:.2f}')

        x, y, z = self.frame.to_xyz(msg.latitude, msg.longitude, msg.altitude)

        out = PointStamped()
        out.header.stamp = msg.header.stamp   # mantém o tempo da medição
        out.header.frame_id = self.frame_id
        out.point.x, out.point.y, out.point.z = x, y, z
        self.pub.publish(out)


def main():
    rclpy.init()
    node = GpsToXYZ()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()
