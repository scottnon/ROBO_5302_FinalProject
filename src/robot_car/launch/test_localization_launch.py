import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    pkg_share = get_package_share_directory('robot_car')
    default_model_path = os.path.join(pkg_share, 'urdf', 'description', 'robot_car_description.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'config.rviz')

    default_map_path = '/home/kingpooper/deepRacerWS/my_map.yaml'
    default_params_path = '/home/kingpooper/deepRacerWS/src/robot_car/config/nav2_params.yaml'

    namespace = LaunchConfiguration('namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    lifecycle_nodes = ['map_server', 'amcl']

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file
    }

    # configured_params = RewrittenYaml(
    #     source_file=params_file,
    #     root_key=namespace,
    #     param_rewrites=param_substitutions,
    #     convert_types=True
    # )

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        DeclareLaunchArgument('namespace', default_value='', description='Top-level namespace'),
        DeclareLaunchArgument('map', default_value=default_map_path, description='Full path to map yaml file to load'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation clock if true'),
        DeclareLaunchArgument('autostart', default_value='true', description='Automatically startup the nav2 stack'),
        DeclareLaunchArgument('params_file', default_value=default_params_path, description='Full path to the ROS2 parameters file to use'),
        DeclareLaunchArgument('model', default_value=default_model_path, description='Absolute path to robot model file'),
        DeclareLaunchArgument('rvizconfig', default_value=default_rviz_config_path, description='Absolute path to rviz config file'),
        DeclareLaunchArgument('gui', default_value='True', description='Flag to enable joint_state_publisher_gui'),

        # Initial pose arguments
        DeclareLaunchArgument('initial_pose_x', default_value='0.0', description='Initial X position'),
        DeclareLaunchArgument('initial_pose_y', default_value='0.0', description='Initial Y position'),
        DeclareLaunchArgument('initial_pose_theta', default_value='0.0', description='Initial orientation (yaw in radians)'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
        ),

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
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rvizconfig')],
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        Node(
            package='robot_car',
            executable='bicycle_model_scott.py',
            name='bicycle_odom',
            output='screen',
            parameters=[{'wheelbase': 0.165}]
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
            parameters=[{
                'use_sim_time': True,
                'yaml_filename': '/home/kingpooper/deepRacerWS/my_map.yaml',
                'map_subscribe_transient_local': True
                }],
        ),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'odom_frame_id': 'odom',
                'base_frame_id': 'base_footprint',
                'global_frame_id': 'map',
                'scan_topic': '/scan',
                'transform_tolerance': 1.0,
                'min_particles': 500,
                'max_particles': 2000,
                'alpha1': 0.2,
                'alpha2': 0.2,
                'alpha3': 0.2,
                'alpha4': 0.2,
                'alpha5': 0.2,
                'laser_model_type': 'likelihood_field',
                'z_hit': 0.5,
                'z_rand': 0.5,
                'z_max': 0.05,
                'z_short': 0.05
            }],
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'autostart': autostart},
                {'node_names': lifecycle_nodes}
            ]
        ),

        # Node(
        #     package='robot_car',
        #     executable='map_metadata_relay',
        #     name='map_metadata_relay',
        # )

        # Publish the initial pose
        # ExecuteProcess(
        #     cmd=[
        #         'ros2', 'topic', 'pub', '--once', '/initialpose', 'geometry_msgs/msg/PoseWithCovarianceStamped',
        #         '{',
        #         'header: {frame_id: "map"},',
        #         'pose: {',
        #         '  pose: {',
        #         '    position: {x: ', LaunchConfiguration('initial_pose_x'), ', y: ', LaunchConfiguration('initial_pose_y'), ', z: 0.0},',
        #         '    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}',
        #         '  },',
        #         '  covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,',
        #         '              0.0, 0.25, 0.0, 0.0, 0.0, 0.0,',
        #         '              0.0, 0.0, 0.0, 0.0, 0.0, 0.0,',
        #         '              0.0, 0.0, 0.0, 0.0, 0.0, 0.0,',
        #         '              0.0, 0.0, 0.0, 0.0, 0.0, 0.0,',
        #         '              0.0, 0.0, 0.0, 0.0, 0.0, 0.0685]',
        #         '}',
        #         '}'
        #     ],
        #     shell=True
        # )
    ])
