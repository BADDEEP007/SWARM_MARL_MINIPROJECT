import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration

def generate_launch_description():
    package_name = 'marl'  # Change this to your package name
    urdf_file_name = 'robot.urdf'  # Change this to your URDF file

    urdf_path = os.path.join(get_package_share_directory(package_name), 'urdf', urdf_file_name)

    # Start Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ])
    )

    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'my_robot', '-file', urdf_path],
        output='screen'
    )

    # Publish URDF to robot_state_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': Command(['cat ', urdf_path])}]
    )

    return LaunchDescription([
        DeclareLaunchArgument(name='use_sim_time', default_value='true', description='Use simulation clock'),
        gazebo_launch,
        spawn_entity,
        robot_state_publisher
    ])
