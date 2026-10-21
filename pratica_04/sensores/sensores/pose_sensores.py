"""Exercício 2.1 — pose do robô medida pelos sensores (GPS + IMU).

Assina:  /gps/xyz   (geometry_msgs/PointStamped)  posição da antena do GPS
         /imu/data  (sensor_msgs/Imu)             giroscópio e orientação
Publica: /odom_sensores   (nav_msgs/Odometry)     pose estimada, frame 'map'
         /pose2d_sensores (geometry_msgs/Pose2D)  mesma pose em (x, y, θ),
                                                  prática pro PlotJuggler
         TF map → base_sensores (opcional)        mostra a pose estimada no
                                                  RViz, ao lado da real

A pose é publicada a cada mensagem da IMU (100 Hz), com o carimbo de tempo
da IMU. O GPS (10 Hz) atualiza a posição; entre duas leituras do GPS a
posição fica constante. É essa /odom_sensores que o trajectory_8 usa como
realimentação no exercício 2.1.

Só para avaliação (opcional): se ground_truth_topic não for vazio, a pose
real do Gazebo (/ground_truth) é republicada em /pose2d_real, para comparar
com /pose2d_sensores no PlotJuggler. Ela NÃO entra na estimativa.
"""
import math

import rclpy
from geometry_msgs.msg import PointStamped, Pose2D, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster

from sensores.estimador import GpsImuEstimator


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class PoseSensores(Node):

    def __init__(self):
        super().__init__('pose_sensores')
        self.declare_parameter('yaw_source', 'gyro')        # gyro | orientation
        self.declare_parameter('gps_offset_x', -0.12)       # antena no base_link (m)
        self.declare_parameter('gps_offset_y', 0.0)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('child_frame_id', 'base_sensores')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('ground_truth_topic', '/ground_truth')

        yaw_source = self.get_parameter('yaw_source').value
        self.estimator = GpsImuEstimator(
            gps_offset=(self.get_parameter('gps_offset_x').value,
                        self.get_parameter('gps_offset_y').value),
            yaw_source=yaw_source)
        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value
        self.tf_broadcaster = (TransformBroadcaster(self)
                               if self.get_parameter('publish_tf').value else None)

        self.odom_pub = self.create_publisher(Odometry, '/odom_sensores', 10)
        self.pose2d_pub = self.create_publisher(Pose2D, '/pose2d_sensores', 10)

        self.create_subscription(Imu, '/imu/data', self.imu_callback,
                                 qos_profile_sensor_data)
        self.create_subscription(PointStamped, '/gps/xyz', self.gps_callback,
                                 qos_profile_sensor_data)

        gt_topic = self.get_parameter('ground_truth_topic').value
        if gt_topic:
            self.real_pub = self.create_publisher(Pose2D, '/pose2d_real', 10)
            self.create_subscription(Odometry, gt_topic, self.ground_truth_callback,
                                     qos_profile_sensor_data)

        self.get_logger().info(f'Pose pelos sensores: GPS (x, y) + IMU (θ: {yaw_source})')

    def gps_callback(self, msg):
        self.estimator.update_gps(msg.point.x, msg.point.y)

    def imu_callback(self, msg):
        t = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        self.estimator.update_imu(t, msg.angular_velocity.z,
                                  yaw_from_quaternion(msg.orientation))
        if not self.estimator.ready():
            return  # ainda sem GPS
        self.publish(msg.header.stamp, *self.estimator.pose(), msg.angular_velocity.z)

    def publish(self, stamp, x, y, theta, wz):
        qz, qw = math.sin(theta / 2.0), math.cos(theta / 2.0)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.angular.z = wz
        self.odom_pub.publish(odom)

        self.pose2d_pub.publish(Pose2D(x=x, y=y, theta=theta))

        if self.tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.frame_id
            tf.child_frame_id = self.child_frame_id
            tf.transform.translation.x = x
            tf.transform.translation.y = y
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf)

    def ground_truth_callback(self, msg):
        p = msg.pose.pose
        self.real_pub.publish(Pose2D(x=p.position.x, y=p.position.y,
                                     theta=yaw_from_quaternion(p.orientation)))


def main():
    rclpy.init()
    node = PoseSensores()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()
