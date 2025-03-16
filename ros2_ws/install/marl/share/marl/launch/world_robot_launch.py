import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Define paths and arguments
    pkg_name = 'marl'  # Replace with your package name
    
    # Get package directory
    pkg_share = get_package_share_directory(pkg_name)
    
    # Set paths for URDF and world file
    default_model_path = os.path.join(pkg_share, 'urdf', 'robot.urdf')
    default_world_path = os.path.join(pkg_share, 'worlds', 'hospital.world')
    
    # Launch Arguments
    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=default_model_path,
        description='Absolute path to robot URDF/XACRO file'
    )
    
    world_arg = DeclareLaunchArgument(
        name='world',
        default_value=default_world_path,
        description='Absolute path to world file'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )
    
    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='true',
        description='Set to "false" to run headless'
    )
    
    # Get launch configurations
    model = LaunchConfiguration('model')
    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    
    # URDF robot state publisher
    robot_description = ParameterValue(
        Command(['xacro ', model]),
        value_type=str
    )
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )
    
    # Joint state publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={
            'world': world,
            'gui': gui,
            'verbose': 'true'
        }.items()
    )
    
    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'my_robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1'
        ],
        output='screen'
    )
    
    # RViz without config file
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(gui)
    )
    
    # Return launch description
    return LaunchDescription([
        model_arg,
        world_arg,
        use_sim_time_arg,
        gui_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        gazebo,
        spawn_entity,
        rviz_node
    ])