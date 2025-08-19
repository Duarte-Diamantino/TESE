#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Diretório do pacote de descrição (URDF)
    pkg_share = get_package_share_directory('pickup_description')
    urdf_path = os.path.join(pkg_share, 'urdf', 'pickup.urdf')
    robot_description = open(urdf_path, 'r').read()

    return LaunchDescription([
        # Publica o URDF em /robot_description e os frames estáticos em /tf_static
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),

        # Publica estados de junta em /joint_states
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
        ),

        # Transforma estática: fixa map -> odom
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            output='screen',
            arguments=[
                '0', '0', '0.1',    # x y z (m)
                '0', '0', '0',      # roll pitch yaw (rad)
                'map',              # frame pai
                'odom'              # frame filho
            ],
        ),

        # Node que publica odom->base_link e /joint_states
        Node(
            package='pickup_description',
            executable='mov_robot_model',
            name='mov_robot_model',
            output='screen',
        ),
    ])
