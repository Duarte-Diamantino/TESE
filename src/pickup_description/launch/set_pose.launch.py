#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, EmitEvent
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1) Declaração de argumentos
    x_arg     = DeclareLaunchArgument('x',    default_value='0.0', description='Odom X in map')
    y_arg     = DeclareLaunchArgument('y',    default_value='0.0', description='Odom Y in map')
    z_arg     = DeclareLaunchArgument('z',    default_value='0.0', description='Odom Z in map')
    roll_arg  = DeclareLaunchArgument('roll', default_value='0.0', description='Odom roll')
    pitch_arg = DeclareLaunchArgument('pitch',default_value='0.0', description='Odom pitch')
    yaw_arg   = DeclareLaunchArgument('yaw',  default_value='0.0', description='Odom yaw')

    # 2) Substituições
    x     = LaunchConfiguration('x')
    y     = LaunchConfiguration('y')
    z     = LaunchConfiguration('z')
    roll  = LaunchConfiguration('roll')
    pitch = LaunchConfiguration('pitch')
    yaw   = LaunchConfiguration('yaw')

    # 3) Node de static_transform_publisher
    static_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='set_odom_pose',
        output='screen',
        arguments=[x, y, z, roll, pitch, yaw, 'map', 'odom']
    )

    # 4) Timer para desligar o launch após 1 segundo
    shutdown = TimerAction(
        period=1.0,
        actions=[EmitEvent(event=Shutdown())]
    )

    return LaunchDescription([
        x_arg, y_arg, z_arg, roll_arg, pitch_arg, yaw_arg,
        static_odom,
        shutdown,
    ])
