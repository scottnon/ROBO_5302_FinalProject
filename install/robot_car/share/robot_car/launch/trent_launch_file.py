from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_share = FindPackageShare(package='robot_car').find('robot_car')
    default_model_path = os.path.join(pkg_share, 'urdf', 'description', 'robot_car_description.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'config.rviz')
    default_nav_params_path = os.path.join(pkg_share, 'config', 'trent_nav_params.yaml')
    default_map_yaml_path = '/home/kingpooper/deepRacerWS/my_map.yaml'

    declare_gui_arg = DeclareLaunchArgument(
        name='gui', default_value='True', description='Use joint_state_publisher_gui'
    )
    declare_model_arg = DeclareLaunchArgument(
        name='model', default_value=default_model_path, description='Robot model URDF file'
    )
    declare_rviz_arg = DeclareLaunchArgument(
        name='rvizconfig', default_value=default_rviz_config_path, description='RViz config file'
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
            parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
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
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', LaunchConfiguration('rvizconfig')],
            output='screen'
        ),

        LifecycleNode(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            namespace='',
            output='screen',
            parameters=[default_nav_params_path]
        ),
        LifecycleNode(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            namespace='',
            output='screen',
            parameters=[default_nav_params_path]
        ),
        LifecycleNode(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            namespace='',
            output='screen',
            parameters=[
                default_nav_params_path,
                {
                    'default_bt_xml_filename': '/home/kingpooper/deepRacerWS/src/robot_car/behavior_trees/navigate_w_replanning_and_recovery.xml'
                }
            ]
        ),


        LifecycleNode(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            namespace='',
            output='screen',
            parameters=[default_nav_params_path]
        ),
        LifecycleNode(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            namespace='',
            output='screen',
            parameters=[default_nav_params_path]
        ),
        LifecycleNode(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            namespace='',
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
                'bond_timeout': 120.0,
                'transition_timeout': 120.0,
                'node_names': [
                    'map_server',
                    'amcl',
                    'bt_navigator',
                    'controller_server',
                    'planner_server',
                    'waypoint_follower'
                ]
            }]
)

    ])
"""[bt_navigator-8] [INFO] [1746418187.452421076] [bt_navigator]: Activating
[bt_navigator-8] [ERROR] [1746418187.453958296] [bt_navigator]: Exception when loading BT: Error at line 12: -> Node not recognized: RemovePassedGoals
[bt_navigator-8] [ERROR] [1746418187.454289810] [bt_navigator]: Error loading XML file: /opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml
[lifecycle_manager-12] [ERROR] [1746418187.492153369] [lifecycle_manager_navigation]: Failed to change state for node: bt_navigator
[lifecycle_manager-12] [ERROR] [1746418187.492292219] [lifecycle_manager_navigation]: Failed to bring up all requested nodes. Aborting bringup."""