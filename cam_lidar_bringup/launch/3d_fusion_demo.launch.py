import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

lidar_pipeline_share_dir = get_package_share_directory('lidar_pipeline')
# Transforms for specific dataset
data_dependant_params = os.path.join(
    lidar_pipeline_share_dir, 'configs', 'may10_config.yaml')
# Lidar detection paramters
pipeline_params = os.path.join(
    lidar_pipeline_share_dir, 'configs', 'lidar_pipeline_config.yaml')
# Yolov8-segmentation model weights
weights_path = os.path.join(
    get_package_share_directory('camera_det3d'), 'yolov8s-seg_half_simplify.engine')

ABS_PATH_TO_ROSBAGS = '/home/adrian/dev/bags/'
BAG_NAME = 'cleaned_bags/may10_q7_clean'

DETECTION_TOPIC = 'image_proc/det3D'
TRACK_TOPIC = 'image_proc/det3D_tracks'

LIDAR_RESULT_ONLY = False
CAM_RESULT_ONLY = False


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

    cam_tracker_node = Node(
        package='obj_tracker',
        executable='object_tracker',
        name='cam_obj_tracker',
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

    lidar_detector = Node(
        package='lidar_pipeline',
        executable='perception_node',
        name='perception_node',
        output='screen',
        parameters=[
            data_dependant_params,
            pipeline_params,
        ]
    )

    lidar_tracker = Node(
        package='obj_tracker',
        executable='object_tracker',
        name='lidar_obj_tracker',
        output='screen',
        parameters=[
            {'detection_topic': '/lidar_proc/o_detections'}
        ]
    )

    lidar_tracker_viz = Node(
        package='obj_tracker',
        executable='tracker_bbox_viz',
        name='tracker_bbox_viz',
        output='screen',
    )

    fusion_3D = Node(
        package='fusion_3d',
        executable='fusion_3d_node',
        name='fusion_3d_node',
        parameters=[
            {'lidar_only_override': LIDAR_RESULT_ONLY},
            {'camera_only_override': CAM_RESULT_ONLY}
        ]
    )

    video_name = BAG_NAME.split('/')[-1]
    fusion_viz = Node(
        package='fusion_3d',
        executable='fusion_3d_viz_node',
        name='fusion_3d_viz_node',
        parameters=[
            {'flip_image': False},
            {'save_video': False},
            {'video_name': video_name},
        ]
    )

    rosbag = ExecuteProcess(
        cmd=['ros2 bag play', ABS_PATH_TO_ROSBAGS + BAG_NAME, '-l'],
        shell=True)

    launch_list = [
        camera_detection3d_node,
        cam_tracker_node,
        tracker_visualization_node,
        lidar_detector,
        lidar_tracker,
        lidar_tracker_viz,
        fusion_3D,
        # fusion_viz,
        rosbag,
    ]

    return LaunchDescription(launch_list)
