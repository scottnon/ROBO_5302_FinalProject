from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    return LaunchDescription([
        Node(   # launch motor listener node
            package='ros2_pca9685',
        #    namespace='motor',
            executable='listener',
         #   name='minimal_subscriber'
        ), 

        Node(   # launch camera node
            package='v4l2_camera',
            executable='v4l2_camera_node',
        ), 

        Node(   # launch tape_follower
            package='tape',
            executable='tape_follower',
        ), 


    ])
    