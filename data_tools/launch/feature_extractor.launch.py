from launch_ros.actions import Node
from launch import LaunchDescription

GT_FOLDER = '/home/adrian/dev/A9_images_and_points/a9_dataset_r02_s01/labels_point_clouds/s110_lidar_ouster_north'
PCD_FOLDER = '/home/adrian/dev/A9_images_and_points/a9_dataset_r02_s01/point_clouds/s110_lidar_ouster_north'
FEATURE_CSV_FOLDER = '/home/adrian/dev/A9_images_and_points/a9_dataset_r02_s01'


def generate_launch_description():

    file2msg = Node(
        package='data_tools',
        executable='extractor',
        name='feature_extractor',
        output='screen',
        parameters=[
                {'gt_folder': GT_FOLDER},
                {'pointcloud_folder': PCD_FOLDER},
                {'output_path': FEATURE_CSV_FOLDER},
        ]
    )

    # Items above will only be launched if they are present in this list
    return LaunchDescription([
        file2msg
    ])
