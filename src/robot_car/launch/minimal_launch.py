from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_share = FindPackageShare(package='robot_car').find('robot_car')
    default_model_path = os.path.join(pkg_share, 'urdf', 'description', 'robot_car_description.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'config.rviz')
    default_nav_params_path = os.path.join(pkg_share, 'config', 'minimal_nav_params.yaml')

    declare_gui_arg = DeclareLaunchArgument(
        name='gui', default_value='True', description='Use joint_state_publisher_gui'
    )
    declare_model_arg = DeclareLaunchArgument(
        name='model', default_value=default_model_path, description='Path to robot model'
    )
    declare_rviz_arg = DeclareLaunchArgument(
        name='rvizconfig', default_value=default_rviz_config_path, description='Path to RViz config'
    )

    return LaunchDescription([
        declare_gui_arg,
        declare_model_arg,
        declare_rviz_arg,

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            condition=UnlessCondition(LaunchConfiguration('gui'))
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            condition=IfCondition(LaunchConfiguration('gui'))
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'use_sim_time': True, 'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
        ),
        Node(
            package='robot_car',
            executable='bicycle_model_scott.py',
            name='bicycle_odom',
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_to_lidar_link_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'lidar_link', 'laser']
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[default_nav_params_path]
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[default_nav_params_path]
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[default_nav_params_path]
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[default_nav_params_path]
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[default_nav_params_path]
        ),
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[default_nav_params_path]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'bond_timeout': 30.0,
                'transition_timeout': 30.0,
                'node_names': [
                    'map_server',
                    'amcl',
                    'controller_server',
                    'planner_server',
                    'bt_navigator',
                    'waypoint_follower'
                ]
            }]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', LaunchConfiguration('rvizconfig')],
            output='screen'
        )
    ])
