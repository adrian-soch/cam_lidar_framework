from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os.path

'''
Options: YOLOv5, YOLOv8, YOLOv9
'''
MODEL_NAME = 'yolov8m.pt'
MODEL_PATH = os.path.join(get_package_share_directory('camera_det2d'), MODEL_NAME)

def generate_launch_description():
    camera_det2d = Node(
        package='camera_det2d',
        executable='camera_processor',
        parameters=[
            {'image_topic': 'image'},
            {'detection_topic': 'image_proc/dets'},
            {'out_image_topic': 'image_proc/result'},
            {'confidence': 0.3},
            {'model_path': MODEL_PATH}
        ]
    )

    return LaunchDescription([camera_det2d])
