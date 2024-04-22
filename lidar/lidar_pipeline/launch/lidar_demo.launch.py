import os

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution

from ament_index_python.packages import get_package_share_directory

from launch_ros.substitutions import FindPackageShare

'''
Rosbag and param file must match for proper functionality
example:
    bag: q7_2_may10_2023
    data_dependant_params file: may10_config.yaml
'''

ABS_PATH_TO_ROSBAGS = '/home/adrian/dev/bags/'
BAG_NAME = 'may10_2023/q7_2_may10_2023'

share_dir = get_package_share_directory('lidar_pipeline')
pipeline_params = os.path.join(
    share_dir, 'configs', 'lidar_pipeline_config.yaml')
data_dependant_params = os.path.join(share_dir, 'configs', 'may10_config.yaml')


def generate_launch_description():

    perception_node = Node(
        package='lidar_pipeline',
        executable='perception_node',
        name='perception_node',
        output='screen',
        parameters=[
                pipeline_params,
                data_dependant_params
        ]
    )

    lidar_tracker = Node(
        package='obj_tracker',
        executable='object_tracker',
        name='lidar_obj_tracker',
        output='screen',
    )

    lidar_tracker_viz = Node(
        package='obj_tracker',
        executable='tracker_bbox_viz',
        name='tracker_bbox_viz',
        output='screen',
    )

    rosbag_play = ExecuteProcess(
        cmd=[[
            'ros2 bag play ',
            ABS_PATH_TO_ROSBAGS,
            BAG_NAME,
            ' -l'
        ]],
        shell=True
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("lidar_pipeline"), "configs", "rviz.rviz"]
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file]
    )

    # Items above will only be launched if they are present in this list
    return LaunchDescription([
        lidar_tracker,
        perception_node,
        lidar_tracker_viz,
        rosbag_play,
        rviz_node
    ])
