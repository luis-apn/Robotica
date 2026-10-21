"""Sobe a simulação da prática 04: Gazebo + robô com sensores.

Argumentos:
  noise_scale (padrão 1.0): multiplica o desvio padrão de TODOS os ruídos dos
                            sensores (exercício 2.1: aumentar o ruído).
  teleop      (padrão true): abre o teleop_twist_keyboard num xterm.
  gui         (padrão true): abre a interface do Gazebo (false = só gzserver).

Exemplo:
  ros2 launch model_description my_robot_urdf.launch.py noise_scale:=5.0 teleop:=false
"""
import os
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Desvio padrão BASE de cada ruído (noise_scale = 1). Unidades:
#   GYRO: rad/s | ACCEL: m/s² | GPS: m | CAMERA: intensidade (0..1) | LIDAR: m
NOISE_STD = {
    'GYRO_STD': 0.005,
    'GYRO_BIAS_STD': 0.0002,
    'ACCEL_STD': 0.02,
    'ACCEL_BIAS_STD': 0.001,
    'GPS_H_STD': 0.05,
    'GPS_V_STD': 0.10,
    'CAMERA_STD': 0.007,
    'LIDAR_STD': 0.01,
}


def robot_description(pkg_share, noise_scale):
    """Lê o URDF, tira os comentários e substitui os placeholders."""
    urdf_file = os.path.join(pkg_share, 'urdf', 'my_robot.urdf')
    yaml_file = os.path.join(pkg_share, 'config', 'diff_drive_controller.yaml')

    with open(urdf_file, 'r') as f:
        urdf = f.read()

    urdf = re.sub(r'<!--.*?-->', '', urdf, flags=re.DOTALL)
    urdf = urdf.replace('$(find model_description)/config/diff_drive_controller.yaml',
                        yaml_file)
    for key, std in NOISE_STD.items():
        urdf = urdf.replace(f'@{key}@', f'{std * noise_scale:.6g}')

    missing = re.findall(r'@[A-Z_]+@', urdf)
    if missing:
        raise RuntimeError(f'Placeholders sem valor no URDF: {set(missing)}')
    return urdf


def launch_setup(context, *args, **kwargs):
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_share = get_package_share_directory('model_description')

    noise_scale = float(LaunchConfiguration('noise_scale').perform(context))
    description = robot_description(pkg_share, noise_scale)
    world_file = os.path.join(pkg_share, 'worlds', 'sensores.world')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world': world_file,
            'gui': LaunchConfiguration('gui'),
        }.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': description, 'use_sim_time': True}],
    )

    # map → odom: o GPS dá a posição num referencial global fixo (map). Como
    # o robô nasce na origem do mundo, map e odom coincidem (TF identidade).
    # Segue a convenção da REP-105: map → odom → base_link.
    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--frame-id', 'map', '--child-frame-id', 'odom'],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'my_robot', '-topic', 'robot_description'],
        output='screen',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller'],
        output='screen',
    )

    load_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    load_diff_drive_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_spawner],
        )
    )

    teleop_keyboard = ExecuteProcess(
        cmd=['xterm', '-e', 'ros2 run teleop_twist_keyboard teleop_twist_keyboard'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('teleop')),
    )

    return [
        gazebo,
        robot_state_publisher,
        map_to_odom,
        spawn_robot,
        load_joint_state_broadcaster,
        load_diff_drive_controller,
        teleop_keyboard,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('noise_scale', default_value='1.0',
                              description='Multiplica o desvio padrão de todos os ruídos'),
        DeclareLaunchArgument('teleop', default_value='true',
                              description='Abre o teleop_twist_keyboard num xterm'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Abre a interface gráfica do Gazebo'),
        OpaqueFunction(function=launch_setup),
    ])
