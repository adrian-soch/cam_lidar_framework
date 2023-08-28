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
current_time = time.strftime("%Y-%m-%d_%H-%M-%S", time.gmtime())
folder_name = ABS_PATH_TO_ROSBAGS + '/' + current_time + '_file2bag'


def generate_launch_description():

    file2msg = Node(
        package='data_tools',
        executable='file2image_cloud',
        name='file2image_cloud',
        output='screen',
        parameters=[
                {'publish_rate': 2.5},
                {'image_folder': '/home/adrian/dev/a9_dataset_r02_s03/images/s110_camera_basler_south1_8mm'},
                {'pointcloud_folder': '/home/adrian/dev/a9_dataset_r02_s03/point_clouds/s110_lidar_ouster_south'},
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
