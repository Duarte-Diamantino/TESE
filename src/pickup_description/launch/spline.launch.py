#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare os parâmetros da sua parábola
    args = [
        DeclareLaunchArgument('a',     default_value='10.0', description='coeficiente a'),
        DeclareLaunchArgument('b',     default_value='10.0', description='coeficiente b'),
        DeclareLaunchArgument('c',     default_value='0.0', description='constante c'),
        DeclareLaunchArgument('x_min', default_value='-5.0', description='x mínimo'),
        DeclareLaunchArgument('x_max', default_value='5.0', description='x máximo'),
        DeclareLaunchArgument('h_offset', default_value='2.0', description='y thing'),
    ]

    return LaunchDescription([
        *args,
        Node(
            package='pickup_description',
            executable='marker_publisher',
            name='spline_marker_publisher',
            output='screen',
            parameters=[{
                'a':      LaunchConfiguration('a'),
                'b':      LaunchConfiguration('b'),
                'c':      LaunchConfiguration('c'),
                'x_min':  LaunchConfiguration('x_min'),
                'x_max':  LaunchConfiguration('x_max'),
                'h_offset':  LaunchConfiguration('h_offset'),
                'frame_id': 'map',  # opcional, padrão do frame
            }],
        ),
    ])
