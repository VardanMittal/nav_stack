import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    # -------------------------------
    # Launch argument
    # -------------------------------
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )

    # -------------------------------
    # Xacro processing
    # -------------------------------
    pkg_path = get_package_share_directory('car_description')
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot_core.xacro')

    robot_description_content = xacro.process_file(xacro_file).toxml()
    robot_description = {'robot_description': robot_description_content}

    # -------------------------------
    # Robot State Publisher
    # -------------------------------
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )

    # -------------------------------
    # ros2_control (Controller Manager)
    # -------------------------------
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            robot_description,
            os.path.join(pkg_path, 'config', 'controller.yaml'),
        ],
        output='screen'
    )

    # -------------------------------
    # Controller Spawners
    # -------------------------------
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    steering_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['steering_controller'],
    )

    rear_wheel_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['rear_wheel_controller'],
    )

    # -------------------------------
    # Launch Description
    # -------------------------------
    return LaunchDescription([
        declare_use_sim_time,
        robot_state_publisher_node,
        controller_manager_node,
        joint_state_broadcaster_spawner,
        steering_controller_spawner,
        rear_wheel_controller_spawner,
    ])
