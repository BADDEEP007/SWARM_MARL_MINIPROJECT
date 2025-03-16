#!/usr/bin/env python3
# ros2_ws/src/marl/marl/marl_agent.py

import rclpy
from rclpy.node import Node
import numpy as np
import tensorflow as tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from gazebo_msgs.msg import ModelStates
import random
import os
import threading
import time
from tf2_ros import TransformListener, Buffer
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class MARLAgent(Node):
    def __init__(self, robot_id, num_robots=5):
        super().__init__(f'marl_agent_{robot_id}')
        
        self.robot_id = robot_id
        self.num_robots = num_robots
        self.robot_name = f"robot_{robot_id}"
        
        # QoS profile for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers and subscribers
        self.vel_pub = self.create_publisher(Twist, f'/{self.robot_name}/cmd_vel', 10)
        self.laser_sub = self.create_subscription(LaserScan, f'/{self.robot_name}/scan', self.laser_callback, sensor_qos)
        self.odom_sub = self.create_subscription(Odometry, f'/{self.robot_name}/odom', self.odom_callback, 10)
        
        # For getting positions of other robots
        self.model_sub = self.create_subscription(ModelStates, '/gazebo/model_states', self.model_callback, 10)
        
        # TF buffer and listener for transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # State variables
        self.laser_data = None
        self.position = None
        self.orientation = None
        self.other_robots_positions = {}
        
        # RL parameters
        self.state_size = 24  # 8 laser readings + 16 for relative positions of other robots
        self.action_size = 5  # Forward, Left, Right, Backward, Stop
        
        # Create RL model
        self.model = self.build_model()
        
        # Replay memory
        self.memory = []
        self.gamma = 0.95  # Discount factor
        self.epsilon = 1.0  # Exploration rate
        self.epsilon_min = 0.01
        self.epsilon_decay = 0.995
        self.learning_rate = 0.001
        
        # Goal
        self.goal_position = np.array([10.0, 10.0])  # Example goal
        
        # Initialize previous state
        self.prev_state = None
        self.prev_action = None
        
        # Create a timer for the control loop
        self.timer = self.create_timer(0.1, self.train)
        
        self.get_logger().info(f"MARL Agent {robot_id} initialized")
    
    def build_model(self):
        model = tf.keras.Sequential([
            tf.keras.layers.Dense(64, activation='relu', input_shape=(self.state_size,)),
            tf.keras.layers.Dense(64, activation='relu'),
            tf.keras.layers.Dense(self.action_size, activation='linear')
        ])
        model.compile(loss='mse', optimizer=tf.keras.optimizers.Adam(learning_rate=self.learning_rate))
        return model
    
    def laser_callback(self, msg):
        # Simplify laser data to 8 directions
        ranges = np.array(msg.ranges)
        ranges = np.nan_to_num(ranges, nan=10.0, posinf=10.0)  # Replace NaN and inf with 10.0
        
        # Take 8 equidistant readings
        step = len(ranges) // 8
        self.laser_data = ranges[::step][:8]
        
    def odom_callback(self, msg):
        self.position = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        ])
        
        # Extract orientation (quaternion)
        self.orientation = np.array([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ])
        
    def model_callback(self, msg):
        # Extract positions of other robots
        for i, name in enumerate(msg.name):
            if name.startswith('robot_') and name != self.robot_name:
                robot_id = int(name.split('_')[1])
                self.other_robots_positions[robot_id] = np.array([
                    msg.pose[i].position.x,
                    msg.pose[i].position.y
                ])
                
    def get_state(self):
        if self.laser_data is None or self.position is None:
            return None
            
        # Prepare laser data
        state = self.laser_data.tolist()
        
        # Add relative positions of other robots (normalized)
        for i in range(self.num_robots):
            if i != self.robot_id and i in self.other_robots_positions:
                rel_pos = self.other_robots_positions[i] - self.position
                # Normalize
                dist = np.linalg.norm(rel_pos)
                if dist > 0:
                    rel_pos = rel_pos / dist
                state.extend([rel_pos[0], rel_pos[1], min(dist, 10.0) / 10.0, 1.0])
            else:
                state.extend([0.0, 0.0, 1.0, 0.0])  # Default values if robot not found
                
        return np.array(state)
        
    def get_reward(self, state, action):
        reward = 0
        
        # Distance to goal reward
        if self.position is not None:
            current_dist = np.linalg.norm(self.position - self.goal_position)
            
            if self.prev_state is not None:
                prev_dist = np.linalg.norm(np.array(self.prev_state[-16:]) - self.goal_position)
                reward += (prev_dist - current_dist) * 10  # Reward for getting closer to goal
                
        # Collision penalty
        if self.laser_data is not None and np.min(self.laser_data) < 0.3:
            reward -= 10  # Penalty for getting too close to obstacles
            
        # Swarm cohesion reward
        if len(self.other_robots_positions) > 0:
            avg_dist = 0
            for pos in self.other_robots_positions.values():
                dist = np.linalg.norm(self.position - pos)
                if dist < 1.0:  # Too close
                    reward -= 2
                elif dist > 5.0:  # Too far
                    reward -= 1
                else:  # Just right
                    reward += 1
                avg_dist += dist
            avg_dist /= len(self.other_robots_positions)
            
            # Ideal swarm distance is between 1.5 and 3 units
            if 1.5 <= avg_dist <= 3.0:
                reward += 2
                
        return reward
        
    def act(self, state):
        if np.random.rand() <= self.epsilon:
            return random.randrange(self.action_size)
        
        act_values = self.model.predict(state.reshape(1, -1))
        return np.argmax(act_values[0])
        
    def step(self, action):
        # Execute action
        move_cmd = Twist()
        
        # Set linear and angular velocities based on action
        if action == 0:  # Forward
            move_cmd.linear.x = 0.2
            move_cmd.angular.z = 0.0
        elif action == 1:  # Left
            move_cmd.linear.x = 0.1
            move_cmd.angular.z = 0.3
        elif action == 2:  # Right
            move_cmd.linear.x = 0.1
            move_cmd.angular.z = -0.3
        elif action == 3:  # Backward
            move_cmd.linear.x = -0.2
            move_cmd.angular.z = 0.0
        elif action == 4:  # Stop
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = 0.0
            
        self.vel_pub.publish(move_cmd)
        
    def train(self):
        state = self.get_state()
        if state is None:
            return
            
        action = self.act(state)
        self.step(action)
        
        next_state = self.get_state()
        if next_state is None:
            return
            
        reward = self.get_reward(next_state, action)
        
        # Store in replay memory
        if self.prev_state is not None and self.prev_action is not None:
            self.memory.append((self.prev_state, self.prev_action, reward, state))
            
        # Update previous state and action
        self.prev_state = state
        self.prev_action = action
        
        # If memory is big enough, learn from experiences
        if len(self.memory) > 32:
            self.replay(32)
            
    def replay(self, batch_size):
        minibatch = random.sample(self.memory, batch_size)
        for state, action, reward, next_state in minibatch:
            target = reward
            if next_state is not None:
                target = reward + self.gamma * np.amax(self.model.predict(next_state.reshape(1, -1))[0])
            target_f = self.model.predict(state.reshape(1, -1))
            target_f[0][action] = target
            self.model.fit(state.reshape(1, -1), target_f, epochs=1, verbose=0)
            
        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay

def main(args=None):
    rclpy.init(args=args)
    
    # Get robot ID from environment or use default
    robot_id = int(os.environ.get('ROBOT_ID', 0))
    num_robots = int(os.environ.get('NUM_ROBOTS', 5))
    
    agent = MARLAgent(robot_id, num_robots)
    
    try:
        rclpy.spin(agent)
    except KeyboardInterrupt:
        pass
    finally:
        agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()