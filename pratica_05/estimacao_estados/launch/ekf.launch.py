"""Sobe o filtro de Kalman (robot_localization) → /odometry/filtered.

Rode DEPOIS da simulação e dos nós de sensores:
  ros2 launch model_description my_robot_urdf.launch.py
  ros2 launch sensores sensores.launch.py
  ros2 launch estimacao_estados ekf.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('estimacao_estados'),
                          'config', 'ekf.yaml')
    return LaunchDescription([
        Node(package='robot_localization', executable='ekf_node',
             name='ekf_filter_node', parameters=[params], output='screen'),
    ])
