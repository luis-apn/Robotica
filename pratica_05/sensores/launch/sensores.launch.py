"""Sobe os nós de processamento dos sensores (+ RViz).

  gps_to_xyz:    /gps/fix → /gps/xyz
  pose_sensores: /gps/xyz + /imu/data → /odom_sensores  (estado bruto)
  rviz2:         câmera, lidar, robô e TF

Rode DEPOIS da simulação (model_description my_robot_urdf.launch.py):
  ros2 launch sensores sensores.launch.py
  ros2 launch sensores sensores.launch.py rviz:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('sensores')
    params = os.path.join(pkg, 'config', 'sensores.yaml')
    rviz_config = os.path.join(pkg, 'rviz', 'sensores.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Abre o RViz com a configuração da prática'),
        DeclareLaunchArgument('yaw_source', default_value='gyro',
                              description='gyro | orientation'),

        Node(package='sensores', executable='gps_to_xyz',
             parameters=[params], output='screen'),

        DeclareLaunchArgument('noise_scale', default_value='1.0',
                              description='O mesmo noise_scale da simulação'),

        # Desvio padrão do GPS informado ao filtro: 0.05 m × noise_scale
        Node(package='sensores', executable='pose_sensores',
             parameters=[params,
                         {'yaw_source': LaunchConfiguration('yaw_source'),
                          'gps_std': ParameterValue(
                              PythonExpression(['0.05 * ', LaunchConfiguration('noise_scale')]),
                              value_type=float)}],
             output='screen'),

        Node(package='rviz2', executable='rviz2',
             arguments=['-d', rviz_config],
             parameters=[{'use_sim_time': True}],
             condition=IfCondition(LaunchConfiguration('rviz')),
             output='screen'),
    ])
