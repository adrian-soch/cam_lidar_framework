import os

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

share_dir = get_package_share_directory('trad_lidar_detector')
pipeline_params = os.path.join(
    share_dir, 'configs', 'lidar_pipeline_config.yaml')
data_dependant_params = os.path.join(share_dir, 'configs', 'may10_config.yaml')


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='trad_lidar_detector',
            executable='perception_node',
            name='perception_node',
            output='screen',
            # prefix='valgrind --leak-check=yes ',
            parameters=[
                pipeline_params,
                data_dependant_params
            ]
        )
    ])
