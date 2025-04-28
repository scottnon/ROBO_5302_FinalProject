from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # find package share
    rplidar_pkg_share = FindPackageShare('rplidar_ros').find('rplidar_ros')

    # build path to package
    rplidar_launch_file = os.path.join(rplidar_pkg_share, 'launch', 'rplidar_a1_launch.py')

    return LaunchDescription([
        
        Node(
            package='ros2_pca9685',
        #    namespace='motor',
            executable='listener',
         #   name='minimal_subscriber'
        ), 
        
        IncludeLaunchDescription(
        	PythonLaunchDescriptionSource(rplidar_launch_file),
        ),

    ])
    
   
