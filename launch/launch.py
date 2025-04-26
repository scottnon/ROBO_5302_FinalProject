from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    return LaunchDescription([
        Node(
            package='ros2_pca9685',
        #    namespace='motor',
            executable='listener',
         #   name='minimal_subscriber'
        ), 
        IncludeLaunchDescription(
        	PythonLaunchDescriptionSource(
        		['/home/kingpooper/deepRacerWS/src/rplidar_ros/launch/rplidar_a1_launch.py']),
        		),

    ])
    
   
