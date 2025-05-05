from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessExit
from launch_ros.descriptions import LifecycleNode

def generate_launch_description():
    diagnostics_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_diagnostics',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': False,
            'bond_timeout': 10.0,
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

    # Helper node to query lifecycle states (nav2 CLI or direct rcl service calls also possible)
    lifecycle_state_printer = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_printer',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': False,
            'bond_timeout': 10.0,
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

    # Timer to trigger after 5s and manually call nav2 lifecycle change-state command
    delay_logging = TimerAction(
        period=5.0,
        actions=[
            LogInfo(msg='----[ LIFECYCLE DEBUG TRIGGERED - check all /get_state services ]----')
        ]
    )

    return LaunchDescription([
        diagnostics_node,
        delay_logging,
        lifecycle_state_printer
    ])
