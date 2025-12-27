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
    # Gazebo Fortress (SYSTEM PROCESS)
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
    # Spawn robot into Gazebo
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
        spawn_robot,
    ])
