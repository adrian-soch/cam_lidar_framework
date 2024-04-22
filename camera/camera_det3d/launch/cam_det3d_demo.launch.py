import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

data_dependant_params = os.path.join(
    get_package_share_directory('trad_lidar_detector'), 'configs', 'may10_config.yaml')
weights_path = os.path.join(
    get_package_share_directory('camera_det3d'), 'yolov8m-seg_half.engine')

print(weights_path)

ABS_PATH_TO_ROSBAGS = '/home/adrian/dev/bags/'
BAG_NAME = 'cleaned_bags/may10_q7_clean'

DETECTION_TOPIC = 'image_proc/det3D'
TRACK_TOPIC = 'image_proc/det3D_tracks'


def generate_launch_description():

    camera_detection3d_node = Node(
        package='camera_det3d',
        executable='camera_det3d',
        name='camera_det3d',
        output='screen',
        parameters=[
            data_dependant_params,
            {'weights': weights_path}],
    )

    tracker_node = Node(
        package='obj_tracker',
        executable='object_tracker',
        name='lidar_obj_tracker',
        output='screen',
        parameters=[
            {'detection_topic': DETECTION_TOPIC},
            {'det_pub_topic': TRACK_TOPIC},
            {'marker_pub_topic': 'image_proc/det3D_track_markers'},
            {'isOBB': True},
        ]
    )

    tracker_visualization_node = Node(
        package='obj_tracker',
        executable='tracker_bbox_viz',
        name='tracker_bbox_viz',
        output='screen',
        parameters=[
            {'topic_name': TRACK_TOPIC},
            {'bbox_marker_topic': 'image_proc/track_bboxes'},
            {'tracklet_topic': 'image_proc/tracklet_marker'},
        ]
    )

    rosbag = ExecuteProcess(
        cmd=['ros2 bag play', ABS_PATH_TO_ROSBAGS + BAG_NAME, '-l'],
        shell=True)

    launch_list = [
        camera_detection3d_node,
        tracker_node,
        tracker_visualization_node,
        rosbag,
    ]

    return LaunchDescription(launch_list)
