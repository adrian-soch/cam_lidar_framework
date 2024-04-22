from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cam2image',
            executable='cam_stream',
            name='brio_camera_node',
            output='screen',
            parameters=[
                {"device_id": 2},
                {"frequency": 10.0},
                {"width": 1920},
                {"height": 1080},
                {"flip": 2}
            ]
        )
     ])
