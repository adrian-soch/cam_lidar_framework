import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

ABS_PATH_TO_FUSION_ENGINE = '/home/adrian/dev/ros2_ws/src/fusion_engine/fusion_engine'
ABS_PATH_TO_ROSBAGS = '/home/adrian/dev/bags'

PACKAGE = 'fusion_engine'

def generate_launch_description():
    # Launch the Brio camera
    brio_camera = ExecuteProcess(
        cmd=[[
            'ros2 run image_tools cam2image --ros-args -p ',
            'device_id:=2 ',
            '-p frequency:=10.0 ',
            '-p width:=1920 ',
            '-p height:=1080'
        ]],
        shell=True
    )
    
    # Add the LiDAR launch file to this launch
    ouster_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(PACKAGE),
            'ouster_driver_launch.py')])
        )
    # Items above will only be launched if they are present in this list
    return LaunchDescription([
        ouster_lidar,
        brio_camera
   ])