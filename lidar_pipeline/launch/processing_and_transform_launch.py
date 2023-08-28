from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    s_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',

        # params from visual inspection
        # To make the road paralell with the XY plane/rviz2 grid
        arguments=['0', '0', '0', '0', '0.2', '0', 'map', 'laser_data_frame']
    )

    s_transform2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',

        # params from visual inspection
        # To make the road paralell with the XY plane/rviz2 grid
        arguments=['0', '0', '0', '3.1416', '0',
                   '0', 'map', 'laser_sensor_frame']
    )

    perception_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution(
                [FindPackageShare("lidar_pipeline"), "launch",
                    "processing_node.launch.py"])]
        )
    )

    # Items above will only be launched if they are present in this list
    return LaunchDescription([
        s_transform2,
        perception_node,
        s_transform
    ])
