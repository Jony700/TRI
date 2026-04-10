from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wall_follower',
            executable='wall_follower_node',
            name='wall_follower',
            output='screen',
            parameters=[{
                'desired_distance': 1.0,
                'base_speed': 0.2,
                'forward_speed': 1.0,
                'max_accel': 0.8,
                'kp': 1.4,
                'ki': 0.01,
                'kd': 1.2,
                'kd_filter': 0.3,
                'front_obstacle_dist': 0.5,
                'scan_topic': 'scan',
                'cmd_vel_topic': 'cmd_vel',
            }],
        ),
    ])