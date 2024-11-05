from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sendbooster_amr_bringup',
            executable='motordriver',
            name='motordriver',
            output='screen'
        ),
        Node(
            package='sendbooster_amr_bringup',
            executable='imu',
            name='imu',
            output='screen'
        ),
        Node(
            package='sendbooster_amr_bringup',
            executable='odometry',
            name='odometry',
            output='screen'
        ),
    ])
