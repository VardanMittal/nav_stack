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

    if not os.path.exists(xacro_file):
        raise FileNotFoundError(f"Xacro file not found: {xacro_file}")

    robot_description_content = xacro.process_file(xacro_file).toxml()
    robot_description = {'robot_description': robot_description_content}

    # -------------------------------
    # Robot State Publisher
    # -------------------------------
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': use_sim_time}
        ]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # -------------------------------
    # Launch Description
    # -------------------------------
    return LaunchDescription([
        declare_use_sim_time,
        robot_state_publisher_node,
        joint_state_publisher_node
    ])
