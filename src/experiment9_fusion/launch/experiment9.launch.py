"""Start the scene, the sensors, the bridge and the fusion node.

This launch is the whole experiment except navigation: you can watch the two
input clouds and the fused cloud here before Nav2 is involved at all.

    ros2 launch experiment9_fusion experiment9.launch.py mode:=solution
    ros2 launch experiment9_fusion experiment9.launch.py gui:=false rviz:=false
"""

import os
from pathlib import Path
import shlex
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from experiment9_core.scene9 import generate


def setup(context):
    value = lambda name: LaunchConfiguration(name).perform(context)

    def boolean(name):
        text = value(name).lower()
        if text not in ('true', 'false'):
            raise ValueError(f'{name} must be true or false')
        return text == 'true'

    exp5 = Path(get_package_share_directory('experiment5_depth'))
    exp7 = Path(get_package_share_directory('experiment7_lidar'))
    share = Path(get_package_share_directory('experiment9_fusion'))
    prefix9 = Path(get_package_prefix('experiment9_fusion'))
    prefix7 = Path(get_package_prefix('experiment7_lidar'))

    # 安装目录和源码对不上时（典型情形：教学包覆盖解压到残留了上一版 build/ 的
    # 工作区，make 判定无需重装），launch 原本只会抛一句
    #   executable 'state_node.py' not found on the libexec directory ...
    # 学生看不出下一步该做什么。这里提前拦住，直接给出要执行的那条命令。
    # 注意用可执行位而不是存在性：--symlink-install 装的是符号链接，源码丢了 +x
    # 时名字还在、却执行不了，ros2 launch 报的仍然是 not found。
    for path in (prefix7 / 'lib/experiment7_lidar/state_node.py',
                 prefix7 / 'lib/experiment7_lidar/preprocess_node.py',
                 prefix9 / 'lib/experiment9_fusion/fusion_node.py'):
        if not path.exists():
            raise RuntimeError(
                f'安装目录里缺少 {path}，说明工作区里的 build/install 还是旧版本。'
                '在实验工作区执行 bash tools/build.sh 重新编译，'
                '脚本会自己清掉旧构建，不需要手工删 build/。')
        if not os.access(path, os.X_OK):
            raise RuntimeError(
                f'{path} 存在但不可执行，通常是对应的源码脚本没有执行位。'
                '在实验工作区执行 bash tools/build.sh，脚本会把 src 下所有 .py/.sh '
                '补成可执行后重新安装。')

    runtime = Path(tempfile.mkdtemp(prefix='experiment9_'))
    # The Livox scan pattern ships with Experiment 7. Reusing it keeps the two
    # experiments looking at the same sensor instead of inventing a third one.
    pattern = exp7 / 'vendor/livox/mid360.csv'
    if not pattern.is_file():
        raise RuntimeError(f'Livox scan pattern missing: {pattern}. Did prepare_workspace.sh run?')
    world = generate(exp5, runtime / 'experiment9.sdf', str(pattern),
                     int(value('samples')), float(value('frequency')))

    robot = ET.parse(exp5 / 'description/r1_depth.urdf')
    robot_description = ET.tostring(robot.getroot(), encoding='unicode')

    actions = [
        SetEnvironmentVariable('PYTHONNOUSERSITE', '1'),
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),
    ]
    for key, paths in (('IGN_GAZEBO_RESOURCE_PATH', [exp5 / 'models']),
                       ('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', [prefix7 / 'lib', prefix9 / 'lib'])):
        joined = os.pathsep.join(str(p) for p in paths)
        current = os.environ.get(key, '')
        actions.append(SetEnvironmentVariable(key, joined + (os.pathsep + current if current else '')))

    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(Path(get_package_share_directory('ros_gz_sim')) / 'launch/gz_sim.launch.py')),
        launch_arguments={
            'gz_args': ('-r -v 2 ' if boolean('gui') else '-s -r -v 2 ') + shlex.quote(world),
            'gz_version': '6',
            'on_exit_shutdown': 'true',
        }.items()))

    actions.append(Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='experiment9_bridge', output='screen',
        parameters=[{'config_file': str(share / 'config/bridge9.yaml'), 'use_sim_time': True}]))

    actions.append(Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'use_sim_time': True,
                     'robot_description': ParameterValue(robot_description, value_type=str)}]))

    # Experiment 7 state node: publishes base_link -> lidar_frame and /livox/imu.
    # publish_truth_tf stays false: the TF chain is map -> odom -> base_link, and
    # a second world -> base_link publisher would make base_link have two parents.
    actions.append(Node(
        package='experiment7_lidar', executable='state_node.py',
        parameters=[{'use_sim_time': True, 'publish_truth_tf': boolean('publish_truth_tf')}]))

    fusion_params = [str(share / 'config/fusion.yaml'),
                     {'use_sim_time': True, 'mode': value('mode')}]
    actions.append(Node(
        package='experiment9_fusion', executable='fusion_node.py', name='experiment9_fusion',
        output='screen', parameters=fusion_params))

    if boolean('rviz'):
        actions.append(Node(
            package='rviz2', executable='rviz2', output='log',
            arguments=['-d', str(share / 'config/experiment9.rviz')],
            parameters=[{'use_sim_time': True}]))
    return actions


def generate_launch_description():
    defaults = {
        'gui': 'true',
        'rviz': 'true',
        'mode': 'student',
        'samples': '12000',
        'frequency': '5.0',
        'publish_truth_tf': 'false',
    }
    return LaunchDescription(
        [DeclareLaunchArgument(key, default_value=value) for key, value in defaults.items()]
        + [OpaqueFunction(function=setup)])
