import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

from controle_trajetoria.utils import yaw_from_quaternion


class OdomMonitor(Node):

    def __init__(self):
        super().__init__('odom_monitor')
        self.declare_parameter('rate', 10.0)
        rate = self.get_parameter('rate').value

        self.pose = None

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.create_timer(1 / rate, self.loop)

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        theta = yaw_from_quaternion(msg.pose.pose.orientation)

        self.pose = (x, y, theta)

    def loop(self):
        if self.pose is None:
            return  # o timer pode disparar antes do primeiro /odom chegar
        x, y, theta = self.pose
        self.get_logger().info(
            f'x={x:.2f}  y={y:.2f}  θ={math.degrees(theta):.1f}°')


def main():
    rclpy.init()
    node = OdomMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
