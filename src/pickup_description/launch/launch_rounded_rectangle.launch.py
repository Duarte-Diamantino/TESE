import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare os parâmetros do retângulo arredondado
    args = [
        DeclareLaunchArgument('width',      default_value='10.0', description='largura do retângulo'),
        DeclareLaunchArgument('height',     default_value='5.0', description='altura do retângulo'),
        DeclareLaunchArgument('radius',     default_value='1.0', description='raio das curvas'),
        DeclareLaunchArgument('center_x',   default_value='0.0', description='posição X do centro'),
        DeclareLaunchArgument('center_y',   default_value='0.0', description='posição Y do centro'),
        DeclareLaunchArgument('frame_id',   default_value='map', description='frame de referência'),
    ]

    return LaunchDescription([
        *args,
        Node(
            package='pickup_description',  # substitua pelo nome do teu pacote
            executable='rounded_rectangle_marker_publisher',
            name='rounded_rectangle_marker_publisher',
            output='screen',
            parameters=[{
                'width':     LaunchConfiguration('width'),
                'height':    LaunchConfiguration('height'),
                'radius':    LaunchConfiguration('radius'),
                'center_x':  LaunchConfiguration('center_x'),
                'center_y':  LaunchConfiguration('center_y'),
                'frame_id':  LaunchConfiguration('frame_id'),
            }],
        ),
    ])

