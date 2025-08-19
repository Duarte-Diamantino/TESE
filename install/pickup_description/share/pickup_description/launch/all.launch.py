#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('pickup_description')
    map_yaml = os.path.join(pkg, 'maps', 'map.yaml')
    urdf_file = os.path.join(pkg, 'urdf', 'pickup.urdf')
    robot_desc = open(urdf_file, 'r').read()

    # 1MapServer em container composable  
    map_container = ComposableNodeContainer(
        name='map_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_isolated',
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_map_server',
                plugin='nav2_map_server::MapServer',
                name='map_server',
                parameters=[{
                    'yaml_filename': map_yaml,
                    'frame_id': 'map',
                    'topic_name': 'map'
                }],
            ),
        ],
        output='screen',
    )

    # 2Lifecycle manager para o MapServer
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server']
        }],
    )

    # 3Publicador de URDF → /robot_description + tf_static
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}],
    )

    # 4Transform estático map → base_link
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_base_link',
        output='screen',
        arguments=[
            '0','0','0',     # x y z
            '0','0','0',     # roll pitch yaw
            'map',           # frame pai
            'base_link'      # frame filho conforme o URDF
        ],
    )

    return LaunchDescription([
        map_container,
        lifecycle_manager,
        robot_state_publisher,
        static_tf,
    ])
