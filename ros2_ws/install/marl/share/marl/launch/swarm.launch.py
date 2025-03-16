# ros2_ws/src/marl/launch/swarm_simulation.launch.py

import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, OpaqueFunction
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import math
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Launch arguments
    num_robots_arg = DeclareLaunchArgument(
        'num_robots',
        default_value='5',
        description='Number of robots in the swarm'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # Path to the package
    
    workspace_path = os.path.expanduser('~/ros2_ws/src')
    pkg_share = get_package_share_directory('marl')
    world_file = "/home/sakina/ros2_ws/install/marl/share/marl/worlds/swarm_world.sdf"
    urdf_path = "/home/sakina/ros2_ws/install/marl/share/marl/urdf/robot.urdf"
    
    # Get parameters
    num_robots = LaunchConfiguration('num_robots')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Start Gazebo
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_init.so', 
             '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )
    
    # URDF robot description
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='urdf')]),
        ' ',
        # urdf_file
    ])
    
    robot_description = {'robot_description': robot_description_content}
    
    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )
    
    # Spawn robots
    def spawn_robots(context):
        num_robots_value = int(context.launch_configurations['num_robots'])
        nodes = []
        
        # Spawn robots in a circle
        radius = 3.0
        for i in range(num_robots_value):
            # Calculate position on circle
            angle = 2 * math.pi * i / num_robots_value
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            yaw = math.atan2(-y, -x)  # Point towards center
            
            # Spawn robot
            spawn_robot_node = Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', f'robot_{i}',
                    '-topic', 'robot_description',
                    '-x', str(x),
                    '-y', str(y),
                    '-z', '0.1',
                    '-Y', str(yaw),
                    '-robot_namespace', f'robot_{i}'
                ],
                output='screen'
            )
            nodes.append(spawn_robot_node)
            
            # Launch MARL agent for this robot
            marl_agent_node = Node(
                package='marl',
                executable='marl_agent',
                name=f'marl_agent_{i}',
                parameters=[
                    {'robot_id': i},
                    {'num_robots': num_robots_value},
                    {'use_sim_time': use_sim_time}
                ],
                output='screen',
                env={'ROBOT_ID': str(i), 'NUM_ROBOTS': str(num_robots_value)}
            )
            nodes.append(marl_agent_node)
        
        return nodes
    
    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'config', 'swarm.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # Swarm coordinator node
    swarm_coordinator_node = Node(
        package='marl',
        executable='swarm_coordinator',
        name='swarm_coordinator',
        parameters=[
            {'num_robots': num_robots},
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )
    
    return LaunchDescription([
        num_robots_arg,
        use_sim_time_arg,
        gazebo,
        robot_state_publisher_node,
        OpaqueFunction(function=spawn_robots),
        rviz_node,
        swarm_coordinator_node
    ])