from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('frame_id', default_value='map'),
        Node(
            package='pickup_description',
            executable='q_path_publisher',
            name='q_path_publisher',
            output='screen',
            parameters=[{'frame_id': LaunchConfiguration('frame_id')}],
        ),
    ])
