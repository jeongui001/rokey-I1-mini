from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    webcam_params = os.path.join(
        get_package_share_directory('webcam_perception'), 'config', 'params.yaml')
    mission_params = os.path.join(
        get_package_share_directory('vehicle_mission'), 'config', 'params.yaml')
    approach_params = os.path.join(
        get_package_share_directory('vehicle_approach'), 'config', 'params.yaml')

    return LaunchDescription([
        Node(
            package='webcam_perception',
            executable='webcam_perception_node',
            name='webcam_perception_node',
            parameters=[webcam_params],
            output='screen',
        ),
        Node(
            package='vehicle_mission',
            executable='vehicle_mission_node',
            name='vehicle_mission_node',
            parameters=[mission_params],
            output='screen',
        ),
        Node(
            package='vehicle_approach',
            executable='vehicle_approach_node',
            name='vehicle_approach_node',
            parameters=[approach_params],
            remappings=[
                ('/tf', '/robot11/tf'),
                ('/tf_static', '/robot11/tf_static'),
            ],
            output='screen',
        ),
    ])
