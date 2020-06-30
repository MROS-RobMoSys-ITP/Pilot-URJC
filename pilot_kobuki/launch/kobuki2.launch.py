from launch import LaunchDescription
from launch_ros.actions import Node
import launch.substitutions
import launch_ros.actions
import os
from pathlib import Path
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            IncludeLaunchDescription, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    hardware_config = Path(get_package_share_directory('pilot_kobuki'), 'config', 'hardware.yaml')
    assert hardware_config.is_file()

    kobuki_dir = get_package_share_directory('pilot_kobuki')
    launch_dir = os.path.join(kobuki_dir, 'launch')



    #enable_align_depth = launch.substitutions.LaunchConfiguration('enable_aligned_depth', default="false")

    #rplidar_dir = get_package_share_directory('rplidar_ros')
    #rplidar_launch = launch.actions.IncludeLaunchDescription(
    #    launch.launch_description_sources.PythonLaunchDescriptionSource(
    #            rplidar_dir + '/launch/rplidar.launch.py'))


    # Create the launch configuration variables
    #namespace = LaunchConfiguration('namespace')
    map_yaml_file = LaunchConfiguration('map')
    #use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    bt_xml_file = LaunchConfiguration('bt_xml_file')
    autostart = LaunchConfiguration('autostart')
    #use_remappings = LaunchConfiguration('use_remappings')


    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(kobuki_dir, 'map', 'test_lab.yaml'), # cambiar por el mapa del bar
        description='Full path to map file to load')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(kobuki_dir, 'params', 'params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_bt_xml_cmd = DeclareLaunchArgument(
        'bt_xml_file',
        default_value=os.path.join(
            get_package_share_directory('nav2_bt_navigator'),
            'behavior_trees', 'navigate_w_replanning_and_recovery.xml'),
        description='Full path to the behavior tree xml file to use')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    #declare_use_remappings_cmd = DeclareLaunchArgument(
    #    'use_remappings', default_value='false',
    #    description='Arguments to pass to all nodes launched by the file')


    #rplidar_node = launch_ros.actions.Node(
            #package='rplidar_ros',
            #node_executable='rplidarNode',

            #parameters=[hardware_config],
            #parameters=[{'enable_aligned_depth':enable_align_depth}],
            #output='screen')
    rplidar_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('rplidar_ros'),
            'launch',
            'rplidar.launch.py'))
        )
    #realsense_node = launch_ros.actions.Node(
    #        package='realsense_ros2_camera',
    #        node_executable='realsense_ros2_camera',
    #        node_name='realsense_ros2_camera',
    #        #parameters=[{'enable_aligned_depth':enable_align_depth}],
    #        output='screen'),


    kobuki_node = launch_ros.actions.Node( package='turtlebot2_drivers', node_executable='kobuki_node', output='screen')

    tf_kobuki2laser_node = launch_ros.actions.Node( package='tf2_ros', node_executable='static_transform_publisher', output='screen',
         arguments=['0.11', '0.0', '0.17',
                 '0', '0', '1', '0',
                 'base_link',
                 'laser_frame'])

    laserfilter_node = launch_ros.actions.Node( package='pilot_kobuki', node_executable='laser_filter_node')

    tf_kobuki2imu_node = launch_ros.actions.Node( package='tf2_ros', node_executable='static_transform_publisher', output='screen',
         arguments=['0.0', '0.0', '0.0',
                 '0', '0', '0', '1',
                 'base_link',
                 'imu_link'])

    nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch',
            'nav2_bringup_launch.py')),
        launch_arguments={
            'autostart': 'true',
            'map': os.path.join(kobuki_dir, 'map', 'test_lab.yaml')
        }.items())



    #turtlebot2_cartographer_prefix = get_package_share_directory('turtlebot2_cartographer')
    #cartographer_config_dir = os.path.join(turtlebot2_cartographer_prefix, 'configuration_files')
    #cartographer_node = launch_ros.actions.Node( package='cartographer_ros', node_executable='cartographer_node', output='screen',
    #    arguments=['-configuration_directory', cartographer_config_dir,
    #    '-configuration_basename', 'turtlebot_2d.lua'
    #]);


    return launch.LaunchDescription([
        rplidar_cmd,
        laserfilter_node,
    #cartographer_node,
    #realsense_node,
        kobuki_node,
        tf_kobuki2laser_node,
        tf_kobuki2imu_node,
        nav2_cmd
])
