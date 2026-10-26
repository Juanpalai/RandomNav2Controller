"""Launch the TurtleBot3 simulation, Nav2, and the random controller."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Create the launch description."""
    headless = LaunchConfiguration('headless')
    use_rviz = LaunchConfiguration('use_rviz')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'tb3_simulation_launch.py',
            ])
        ),
        launch_arguments={
            'headless': headless,
            'use_rviz': use_rviz,
        }.items(),
    )

    controller = Node(
        package='turtlebot3_nav2',
        executable='nav2_controller',
        name='nav2_controller',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'headless',
            default_value='False',
            description='Run Gazebo without the graphical client',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='True',
            description='Start RViz with the Nav2 configuration',
        ),
        simulation,
        controller,
    ])
