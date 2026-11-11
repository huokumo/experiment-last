"""Full loop: scene + fusion + map + Nav2. Send a goal, watch the robot drive.

    ros2 launch experiment9_fusion nav_demo.launch.py mode:=solution

The map, the Nav2 parameters and the TF chain are all pre-set. The only thing
that changes between a working and a broken run is the fused cloud on
/r1/fusion/points, which is the point of the exercise.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

LIFECYCLE_NODES = ['map_server', 'planner_server', 'controller_server',
                   'behavior_server', 'bt_navigator']


def generate_launch_description():
    share = Path(get_package_share_directory('experiment9_fusion'))
    params = str(share / 'config/nav2_params.yaml')
    map_yaml = str(share / 'config/map/warehouse.yaml')

    args = [
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('mode', default_value='student'),
        DeclareLaunchArgument('autostart', default_value='true'),
    ]

    scene = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / 'launch/experiment9.launch.py')),
        launch_arguments=[('gui', LaunchConfiguration('gui')),
                          ('rviz', LaunchConfiguration('rviz')),
                          ('mode', LaunchConfiguration('mode'))])

    # map -> odom is identity: the simulator odometry has no drift, so there is
    # nothing for a localiser to correct. Swapping in a real SLAM source later
    # means replacing this one transform, not touching the fusion chain.
    map_to_odom = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='experiment9_map_to_odom',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'map', '--child-frame-id', 'odom'])

    odom_tf = Node(package='experiment9_fusion', executable='odom_tf.py', output='screen',
                   parameters=[{'use_sim_time': True}])

    nodes = [
        Node(package='nav2_map_server', executable='map_server', name='map_server', output='screen',
             parameters=[params, {'yaml_filename': map_yaml, 'use_sim_time': True}]),
        Node(package='nav2_planner', executable='planner_server', name='planner_server', output='screen',
             parameters=[params], remappings=[('cmd_vel', '/r1/cmd_vel')]),
        Node(package='nav2_controller', executable='controller_server', name='controller_server', output='screen',
             parameters=[params], remappings=[('cmd_vel', '/r1/cmd_vel')]),
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server', output='screen',
             parameters=[params], remappings=[('cmd_vel', '/r1/cmd_vel')]),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator', output='screen',
             parameters=[params]),
    ]

    lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager',
        output='screen',
        parameters=[{'use_sim_time': True,
                     'autostart': ParameterValue(LaunchConfiguration('autostart'), value_type=bool),
                     'node_names': LIFECYCLE_NODES}])

    return LaunchDescription(args + [scene, map_to_odom, odom_tf] + nodes + [lifecycle])
