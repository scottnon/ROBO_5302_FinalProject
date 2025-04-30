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
    default_slam_params_path = os.path.join(pkg_share, 'config', 'slam_toolbox_params.yaml')

    # Declare launch arguments
    declare_slam_arg = DeclareLaunchArgument(
        name='slam', default_value='False', description='Run SLAM Toolbox'
    )
    declare_gui_arg = DeclareLaunchArgument(
        name='gui', default_value='True', description='Flag to enable joint_state_publisher_gui'
    )
    declare_model_arg = DeclareLaunchArgument(
        name='model', default_value=default_model_path, description='Absolute path to robot model file'
    )
    declare_rviz_arg = DeclareLaunchArgument(
        name='rvizconfig', default_value=default_rviz_config_path, description='Absolute path to rviz config file'
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
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[default_slam_params_path],
        condition=IfCondition(LaunchConfiguration('slam'))
    )

    return LaunchDescription([
        declare_slam_arg,
        declare_gui_arg,
        declare_model_arg,
        declare_rviz_arg,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node,
        slam_toolbox_node
    ])
