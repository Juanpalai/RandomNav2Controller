from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav2_bringup',
            executable='bringup_launch.py.py',
            output='screen',
            parameters=[
                {'use_sim_time': True}
            ]
        ),
        Node(
            package='turtlebot3_nav2',
            executable='nav2_controller',
            name='nav2_controller',
            output='screen'
        )
    ])
