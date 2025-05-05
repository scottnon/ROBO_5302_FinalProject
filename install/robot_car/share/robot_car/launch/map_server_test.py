from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'yaml_filename': '/home/kingpooper/deepRacerWS/my_map.yaml',
                'frame_id': 'map',
                'always_send_full_map': True,
                'use_lifecycle_node': False
            }]
        ),

        Node(
            package='robot_car',
            executable='map_metadata_relay',
            name='map_metadata_relay',
            output='screen',
        ),
    ])

    #eog /home/kingpooper/deepRacerWS/my_map.pgm
