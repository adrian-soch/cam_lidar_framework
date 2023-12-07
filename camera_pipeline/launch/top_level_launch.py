import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

ABS_PATH_TO_CAMERA_PIPELINE = '/home/adrian/dev/ros2_ws/src/camera_pipeline/camera_pipeline'
ABS_PATH_TO_ROSBAG = 'PATH_HERE'

PACKAGE = 'camera_pipeline'


def generate_launch_description():

    # Start rviz with the desired configuration file
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d' + os.path.join(
            get_package_share_directory(PACKAGE),
            'brio_ouster_config_rviz.rviz')])

    # Run the camera processing script in the correct folder
    #   becuase there are many includes and colcon build doesnt like
    #   running it as a fomrmal package
    execute_camera_processor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(PACKAGE),
            'cam_processor_launch.py')])
    )

    # Play the rosbag in the supplied folder
    rosbag_play = ExecuteProcess(
        cmd=[[
            'ros2 bag play ',
            ABS_PATH_TO_ROSBAG,
            ' -l'
        ]],
        shell=True
    )

    # Items above will only be launched if they are present in this list
    return LaunchDescription([
        rviz_node,
        execute_camera_processor,
        rosbag_play
    ])
