import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch the Brio camera
    brio_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            FindPackageShare("cam2image"),
            "camera_launch.py")])
    )

    # Add the LiDAR launch file to this launch
    ouster_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            FindPackageShare("camera_pipeline"),
            'ouster_driver_launch.py')])
    )
    # Items above will only be launched if they are present in this list
    return LaunchDescription([
        ouster_lidar,
        brio_camera
    ])
