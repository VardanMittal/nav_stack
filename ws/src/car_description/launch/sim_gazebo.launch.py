import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():

    pkg_path = get_package_share_directory('car_description')
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot_core.xacro')

    robot_description = {
        'robot_description': xacro.process_file(xacro_file).toxml()
    }

    # Gazebo Harmonic
    gazebo = Node(
        package='ros_gz_sim',
        executable='gz_sim',
        arguments=['-r', 'empty.sdf'],
        output='screen'
    )

    # Spawn robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'neuro_nav',
            '-topic', 'robot_description'
        ],
        output='screen'
    )

    # robot_state_publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        rsp,
        spawn_robot,
    ])
