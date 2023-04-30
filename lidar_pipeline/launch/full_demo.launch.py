from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare

ABS_PATH_TO_ROSBAGS = '/home/adrian/dev/bags'

def generate_launch_description():

    s_transform = Node(
            package='tf2_ros',
            executable='static_transform_publisher',

            #params from visual inspection
            # To make the road paralell with the XY plane/rviz2 grid
            arguments = ['0', '0', '0', '0', '0.2', '0', 'map', 'laser_data_frame']
        )

    s_transform2 = Node(
            package='tf2_ros',
            executable='static_transform_publisher',

            #params from visual inspection
            # To make the road paralell with the XY plane/rviz2 grid
            arguments = ['0', '0', '0', '3.1416', '0', '0', 'map', 'laser_sensor_frame']
        )

    lidar_tracker = Node(
            package='obj_tracker',
            executable='object_tracker',
            name='lidar_obj_tracker',
            output='screen',
        )

    perception_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution(
                [FindPackageShare("lidar_pipeline"), "launch",
                    "processing_and_transform_launch.py"])]
        )
    )

    rosbag_play = ExecuteProcess(
        cmd=[[
            'ros2 bag play ',
            ABS_PATH_TO_ROSBAGS,
            '/dec7_2022/roofTestDark_1_HD_qosOverrride_true/',
            ' -l'
        ]],
        shell=True
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("lidar_pipeline"), "configs", "rviz.rviz"]
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file]
    )
    
    # Items above will only be launched if they are present in this list
    return LaunchDescription([
        s_transform2,
        lidar_tracker,
        perception_node,
        s_transform,
        rosbag_play,
        rviz_node
   ])