import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _append_source(layer: dict, source_name: str) -> None:
    sources = str(layer.get('observation_sources', '')).split()
    if source_name not in sources:
        sources.append(source_name)
    layer['observation_sources'] = ' '.join(source for source in sources if source)


def _setup(context):
    namespace = LaunchConfiguration('namespace').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    obstacle_topic = LaunchConfiguration('obstacle_topic').perform(context)

    share = get_package_share_directory('turtlebot4_navigation')
    default_yaml = os.path.join(share, 'config', 'nav2.yaml')
    official_launch = os.path.join(share, 'launch', 'nav2.launch.py')

    with open(default_yaml, 'r', encoding='utf-8') as stream:
        params = yaml.safe_load(stream)

    source = {
        'topic': obstacle_topic,
        'data_type': 'PointCloud2',
        'marking': True,
        'clearing': True,
        'min_obstacle_height': 0.04,
        'max_obstacle_height': 1.20,
        'obstacle_min_range': 0.15,
        'obstacle_max_range': 2.50,
        'raytrace_min_range': 0.10,
        'raytrace_max_range': 3.00,
        'observation_persistence': 0.0,
    }

    local_layer = params['local_costmap']['local_costmap']['ros__parameters']['voxel_layer']
    _append_source(local_layer, 'oakd_points')
    local_layer['oakd_points'] = dict(source)

    global_layer = params['global_costmap']['global_costmap']['ros__parameters']['obstacle_layer']
    _append_source(global_layer, 'oakd_points')
    global_layer['oakd_points'] = dict(source)

    output_dir = os.path.join(tempfile.gettempdir(), 'b3_int1_mini_nav2')
    os.makedirs(output_dir, exist_ok=True)
    patched_yaml = os.path.join(output_dir, f'nav2_oakd_{os.getpid()}.yaml')
    with open(patched_yaml, 'w', encoding='utf-8') as stream:
        yaml.safe_dump(params, stream, sort_keys=False)

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(official_launch),
            launch_arguments={
                'namespace': namespace,
                'use_sim_time': use_sim_time,
                'params_file': patched_yaml,
            }.items(),
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='/robot11'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'obstacle_topic',
            default_value='/vehicle_approach/obstacle_points',
        ),
        OpaqueFunction(function=_setup),
    ])
