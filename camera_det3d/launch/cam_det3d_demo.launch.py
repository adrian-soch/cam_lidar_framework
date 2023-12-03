import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

share_dir = get_package_share_directory('lidar_pipeline')
data_dependant_params = os.path.join(share_dir, 'configs', 'may10_config.yaml')

print(data_dependant_params)

ABS_PATH_TO_ROSBAGS = '/home/adrian/dev/bags/'
BAG_NAME = 'cleaned_bags/may10_q7_clean'

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_det3d',
            executable='camera_det3d',
            name='camera_det3d',
            output='screen',
            parameters=[data_dependant_params],
        ),

        ExecuteProcess(
            cmd=['ros2 bag play', ABS_PATH_TO_ROSBAGS + BAG_NAME,'-l'],
            shell=True)
    ])
