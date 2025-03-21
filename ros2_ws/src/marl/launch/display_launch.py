import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'marl'  # Change this to your package name
    urdf_file_name = 'robot.urdf'  # Change this to your URDF file name

    urdf_file_path = os.path.join(get_package_share_directory(package_name), 'urdf', urdf_file_name)

    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': Command(['cat ', urdf_file_path])}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', os.path.join(get_package_share_directory(package_name), 'rviz', 'default.rviz')]
        )
    ])
