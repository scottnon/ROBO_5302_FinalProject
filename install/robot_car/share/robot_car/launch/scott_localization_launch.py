from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_share = FindPackageShare('robot_car').find('robot_car')
    default_model_path = os.path.join(pkg_share, 'urdf', 'description', 'robot_car_description.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'config.rviz')
    default_slam_params_path = os.path.join(pkg_share, 'config', 'mapper_params_offline.yaml')
    default_nav2_params_path = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    default_map_path = os.path.join(pkg_share, 'maps', 'my_map.yaml')

    # Launch arguments
    declare_model_arg = DeclareLaunchArgument(
        name='model', default_value=default_model_path, description='Absolute path to robot model file'
    )
    declare_rviz_arg = DeclareLaunchArgument(
        name='rvizconfig', default_value=default_rviz_config_path, description='Absolute path to rviz config file'
    )
    declare_slam_params_arg = DeclareLaunchArgument(
        name='slam_params_file', default_value=default_slam_params_path, description='SLAM Toolbox params (offline)'
    )
    declare_nav2_params_arg = DeclareLaunchArgument(
        name='nav2_params_file', default_value=default_nav2_params_path, description='Nav2 config params'
    )
    declare_map_arg = DeclareLaunchArgument(
        name='map', default_value=default_map_path, description='Path to the saved map file'
    )

    # Nodes
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui', default='True'))
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui', default='True'))
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    odom_node = Node(
        package='robot_car',
        executable='bicycle_model_scott.py',
        name='bicycle_odom',
        output='screen'
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[LaunchConfiguration('slam_params_file')]
    )

    nav2_bringup = Node(
        package='nav2_bringup',
        executable='bringup_launch.py',
        name='nav2_bringup',
        output='screen',
        arguments=['--ros-args', '--param', 'use_sim_time:=false'],
        parameters=[LaunchConfiguration('nav2_params_file')],
        remappings=[('map', LaunchConfiguration('map'))]
    )

    lidar_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_to_lidar_link_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'lidar_link', 'laser']
    )

    return LaunchDescription([
        declare_model_arg,
        declare_rviz_arg,
        declare_slam_params_arg,
        declare_nav2_params_arg,
        declare_map_arg,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        odom_node,
        slam_toolbox_node,
        nav2_bringup,
        rviz_node,
        lidar_to_laser
    ])
