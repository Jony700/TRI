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
                'forward_speed': 4.0,
                'accel': 0.4,
                'kp': 1.5,
                'kd': 0.5,
                'front_obstacle_dist': 0.5,
                'scan_topic': 'scan',
                'cmd_vel_topic': 'cmd_vel',
            }],
        ),
    ])
