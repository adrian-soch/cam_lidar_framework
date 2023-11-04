'''
This launches 2 nodes:

1) a node that reads from a folder of pcd files and a folder of
images and publishes them.

2) Starts a rosbag2 bag recording 
'''
import time

from launch import LaunchDescription

from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.actions import TimerAction

ABS_PATH_TO_ROSBAGS = '/home/adrian/dev/bags/cleaned_bags/'
IMAGE_FOLDER = '/home/adrian/dev/bags/may10_q7_rebag/images'
PCD_FOLDER = '/home/adrian/dev/bags/may10_q7_rebag/pcds'

current_time = time.strftime("%Y-%m-%d_%H-%M-%S", time.gmtime())
folder_name = ABS_PATH_TO_ROSBAGS + '/' + 'may10_q7_clean_rate_test'

START_DELAY = 1.5
PUBLISH_RATE = 10.0

# ABS_PATH_TO_ROSBAGS + '/' + \
# current_time + '_' + PCD_FOLDER.split('/')[-3]


def generate_launch_description():

    recorder = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '/points', '/image',
             '--max-cache-size', str(int(float(2e8))), '-o', folder_name],
        output='screen'
    )

    file2msg = Node(
        package='data_tools',
        executable='file2image_cloud',
        name='file2image_cloud',
        output='screen',
        parameters=[
                {'publish_rate': PUBLISH_RATE},
                {'image_folder': IMAGE_FOLDER},
                {'pointcloud_folder': PCD_FOLDER},
        ]
    )

    delayed_pub = TimerAction(
        period=START_DELAY,
        actions=[
            file2msg
        ],
    )

    # Items above will only be launched if they are present in this list
    return LaunchDescription([
        recorder,
        delayed_pub,
    ])
