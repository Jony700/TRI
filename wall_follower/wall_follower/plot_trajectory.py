#!/usr/bin/env python3
"""Records robot odometry and generates a trajectory plot upon exit."""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import sys

class TrajectoryPlotter(Node):
    def __init__(self):
        super().__init__('trajectory_plotter')
        self.sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.x_data = []
        self.y_data = []
        self.get_logger().info('Plotter started. Move the robot. Press Ctrl+C to save the plot.')

    def odom_callback(self, msg: Odometry):
        self.x_data.append(msg.pose.pose.position.x)
        self.y_data.append(msg.pose.pose.position.y)

    def save_plot(self):
        if not self.x_data:
            self.get_logger().warn('No data received. Cannot plot.')
            return
            
        plt.figure(figsize=(8, 6))
        plt.plot(self.x_data, self.y_data, '-b', linewidth=2, label='Robot Trajectory')
        plt.plot(self.x_data[0], self.y_data[0], 'go', markersize=8, label='Start')
        plt.plot(self.x_data[-1], self.y_data[-1], 'rx', markersize=8, label='End')
        
        # Plot the theoretical center of the circular room based on assignment1.sdf
        plt.plot(3.0, 2.0, marker='*', color='orange', markersize=12, linestyle='None', label='Circle Center')
        
        plt.xlabel('X Coordinate (m)')
        plt.ylabel('Y Coordinate (m)')
        plt.title('Robot Navigation Trajectory')
        plt.legend()
        plt.grid(True)
        plt.axis('equal')
        
        plt.savefig('trajectory.png', dpi=300)
        self.get_logger().info('Plot successfully saved as trajectory.png')

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlotter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_plot()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()