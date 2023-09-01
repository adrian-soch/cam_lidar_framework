import time

from launch_ros.actions import Node
from launch import LaunchDescription

GT_FOLDER = '/home/adrian/dev/A9_images_and_points/a9_dataset_r02_s03/labels_point_clouds/s110_lidar_ouster_south'
PCD_FOLDER = '/home/adrian/dev/A9_images_and_points/a9_dataset_r02_s03/point_clouds/s110_lidar_ouster_south'

current_time = time.strftime("%Y-%m-%d_%H-%M-%S", time.gmtime())
folder_name = '/home/adrian/dev/metrics/extracted_features' + \
    current_time + '_' + PCD_FOLDER.split('/')[-3]


def generate_launch_description():

    file2msg = Node(
        package='data_tools',
        executable='extractor',
        name='feature_extractor',
        output='screen',
        parameters=[
                {'gt_folder': GT_FOLDER},
                {'pointcloud_folder': PCD_FOLDER},
        ]
    )

    # Items above will only be launched if they are present in this list
    return LaunchDescription([
        file2msg
    ])
