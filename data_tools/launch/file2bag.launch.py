'''
This launches 2 nodes:

1) a node that reads from a folder of pcd files and a folder of
images and publishes them.

2) Starts a rosbag2 bag recording 
'''
import time

from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch import LaunchDescription

ABS_PATH_TO_ROSBAGS = '/home/adrian/dev/bags/'
IMAGE_FOLDER = '/home/adrian/dev/A9_images_and_points/a9_dataset_r02_s03/images/s110_camera_basler_south1_8mm'
PCD_FOLDER = '/home/adrian/dev/A9_images_and_points/a9_dataset_r02_s03/point_clouds/s110_lidar_ouster_south'

current_time = time.strftime("%Y-%m-%d_%H-%M-%S", time.gmtime())
folder_name = ABS_PATH_TO_ROSBAGS + '/' + \
    current_time + '_' + PCD_FOLDER.split('/')[-3]


def generate_launch_description():

    file2msg = Node(
        package='data_tools',
        executable='file2image_cloud',
        name='file2image_cloud',
        output='screen',
        parameters=[
                {'publish_rate': 2.5},
                {'image_folder': IMAGE_FOLDER},
                {'pointcloud_folder': PCD_FOLDER},
        ]
    )

    recorder = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '/points', '/images', '-o', folder_name],
        output='screen'
    )

    # Items above will only be launched if they are present in this list
    return LaunchDescription([
        file2msg,
        recorder
    ])
