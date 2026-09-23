import os
import re

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_model_description = get_package_share_directory('model_description')

    urdf_file = os.path.join(pkg_model_description, 'urdf', 'my_robot.urdf')
    yaml_file = os.path.join(pkg_model_description, 'config', 'diff_drive_controller.yaml')

    with open(urdf_file, 'r') as infp:
        robot_description_raw = infp.read()

    robot_description_raw = re.sub(r'<!--.*?-->', '', robot_description_raw, flags=re.DOTALL)

    robot_description = robot_description_raw.replace(
        '$(find model_description)/config/diff_drive_controller.yaml',
        yaml_file
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        )
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'my_robot', '-topic', 'robot_description'],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller'],
        output='screen'
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
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
        load_joint_state_broadcaster,
        load_diff_drive_controller,
        teleop_keyboard,
    ])
