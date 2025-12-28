import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():

    pkg_path = get_package_share_directory('car_description')
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot_core.xacro')

    robot_description = {
        'robot_description': xacro.process_file(xacro_file).toxml()
    }

    # -------------------------------
    # Gazebo Fortress
    # -------------------------------
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', 'empty.sdf', '-r'],
        output='screen'
    )

    # -------------------------------
    # Robot State Publisher
    # -------------------------------
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen'
    )

    # -------------------------------
    # ros2_control
    # -------------------------------
    controller_manager = Node(
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
    jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    steering = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['steering_controller'],
    )

    rear_wheels = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['rear_wheel_controller'],
    )

    # -------------------------------
    # Spawn Robot into Gazebo
    # -------------------------------
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'neuro_nav',
            '-topic', 'robot_description'
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        rsp,
        controller_manager,
        jsb,
        steering,
        rear_wheels,
        spawn_robot,
    ])
