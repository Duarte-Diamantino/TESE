import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode

def generate_launch_description():
    # --- Declare launch args ---
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (or /clock) if true'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    # --- Resolve package paths ---
    pkg_share = get_package_share_directory('pickup_description')
    urdf_path        = os.path.join(pkg_share, 'urdf',    'pickup.urdf')
    map_yaml_path    = os.path.join(pkg_share, 'maps',    'map.yaml')
    rviz_config_path = os.path.join(pkg_share, 'config',  'display.rviz')

    # --- Nodes ---
    # 1map_server com ciclo de vida
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='',
        output='screen',
        parameters=[
            {'yaml_filename':   map_yaml_path},
        ],
    )

    # 2lifecycle manager para o map_server
    lifecycle_mgr = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart':    True},
            {'node_names':  ['map_server']},
        ],
    )

    # 3static transform map → base_link (identidade)
    static_map_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_base',
        arguments=['0','0','0','0','0','0','map','base_link'],
    )

    # 4robot_state_publisher para o URDF
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time':    use_sim_time},
            {'robot_description': open(urdf_path).read()},
        ],
    )

    # 5rviz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path] if os.path.exists(rviz_config_path) else [],
    )

    return LaunchDescription([
        # launch args
        declare_use_sim_time,

        # nodes
        map_server,
        lifecycle_mgr,
        static_map_to_base,
        robot_state_pub,
        rviz,
    ])
