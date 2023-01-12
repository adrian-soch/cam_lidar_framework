from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lidar_pipeline',
            executable='lidar_node',
            name='lidar_node',
            output='screen',
            # parameters=[
            #     {"cloud_topic": "/kinect/depth_registered/points"},
            #     {"world_frame": "world_frame"},
            #     {"camera_frame": "kinect_link"},
            # ]
        )
     ])