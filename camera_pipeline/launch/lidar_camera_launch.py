import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Launch the Brio camera
    brio_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory("cam2image"), "launch",
            "camera_launch.py")])
    )

    # Add the LiDAR launch file to this launch
    ouster_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory("camera_pipeline"),
            "ouster_driver_launch.py")])
    )
    # Items above will only be launched if they are present in this list
    return LaunchDescription([
        brio_camera,
        ouster_lidar
    ])
