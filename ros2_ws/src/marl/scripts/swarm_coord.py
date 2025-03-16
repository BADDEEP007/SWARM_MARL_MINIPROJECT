#!/usr/bin/env python3
# ros2_ws/src/marl/marl/swarm_coordinator.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import Pose, PoseArray
import numpy as np
import time

class SwarmCoordinator(Node):
    def __init__(self):
        super().__init__('swarm_coordinator')
        
        # Get parameters
        self.declare_parameter('num_robots', 5)
        self.num_robots = self.get_parameter('num_robots').value
        
        # Create publishers and subscribers
        self.swarm_state_pub = self.create_publisher(PoseArray, '/swarm/state', 10)
        self.swarm_goal_pub = self.create_publisher(Pose, '/swarm/goal', 10)
        
        # Create subscribers for each robot's status
        self.robot_status_subs = []
        for i in range(self.num_robots):
            sub = self.create_subscription(
                String,
                f'/robot_{i}/status',
                lambda msg, robot_id=i: self.robot_status_callback(msg, robot_id),
                10
            )
            self.robot_status_subs.append(sub)
        
        # Dictionary to store robot statuses
        self.robot_statuses = {}
        
        # Create goal position
        self.goal_pose = Pose()
        self.goal_pose.position.x = 8.0
        self.goal_pose.position.y = 8.0
        self.goal_pose.position.z = 0.1
        
        # Publish goal position periodically
        self.goal_timer = self.create_timer(1.0, self.publish_goal)
        
        # Monitor swarm state periodically
        self.state_timer = self.create_timer(0.1, self.publish_swarm_state)
        
        self.get_logger().info('Swarm Coordinator initialized with %d robots' % self.num_robots)
    
    def robot_status_callback(self, msg, robot_id):
        self.robot_statuses[robot_id] = msg.data
        
    def publish_goal(self):
        self.swarm_goal_pub.publish(self.goal_pose)
        self.get_logger().debug('Published swarm goal')
        
    def publish_swarm_state(self):
        # This method would collect data from all robots and publish aggregated swarm state
        # In a real implementation, we'd get this data from subscriptions or tf
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = 'world'
        
        # In a real implementation, we would populate this with actual robot poses
        self.swarm_state_pub.publish(pose_array)

def main(args=None):
    rclpy.init(args=args)
    
    coordinator = SwarmCoordinator()
    
    try:
        rclpy.spin(coordinator)
    except KeyboardInterrupt:
        pass
    finally:
        coordinator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()  