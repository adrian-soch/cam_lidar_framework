import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

rviz_config_file = PathJoinSubstitution(
    [FindPackageShare('cam_lidar_bringup'), 'configs', '3d_demo_config.rviz']
)

ABS_PATH_TO_ROSBAGS = '/media/adrian/ed5feae6-7808-425e-b80f-f8bc175919dc/home/adrian/dev/bags/'

DETECTION_TOPIC = 'image_proc/det3D'
TRACK_TOPIC = 'image_proc/det3D_tracks'

LIDAR_RESULT_ONLY = False
CAM_RESULT_ONLY = False
BAG_NAME = ''
CONFIG = ''
PLAY_RATE = 1

DEMO_ID = 1

if DEMO_ID == 1:
    BAG_NAME = '2023-08-30_13-58-46_a9_dataset_r02_s03_camSouth1_LidarSouth'
    CONFIG = 'r02_s03_cam1South_lidarSouth_config.yaml'
    PLAY_RATE = 4
if DEMO_ID == 2:
    # Height is too low, thus, angle is too horizontal for this 3d cam method to work
    BAG_NAME = 'dec14_2023/dec14_2023_good'
    CONFIG = 'dec14_config.yaml'

lidar_pipeline_share_dir = get_package_share_directory('trad_lidar_detector')
# Transforms for specific dataset
data_dependant_params = os.path.join(
    lidar_pipeline_share_dir, 'configs', CONFIG)
# Lidar detection paramters
pipeline_params = os.path.join(
    lidar_pipeline_share_dir, 'configs', 'lidar_pipeline_config.yaml')

# Yolov8-segmentation model weights
weights_path = os.path.join(
    get_package_share_directory('camera_det3d'), 'yolov8s-seg.pt')


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
        package='trad_lidar_detector',
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
        cmd=['ros2 bag play', ABS_PATH_TO_ROSBAGS +
             BAG_NAME, '-l', '-r', str(PLAY_RATE)],
        shell=True)

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file]
    )

    launch_list = [
        camera_detection3d_node,
        cam_tracker_node,
        tracker_visualization_node,
        lidar_detector,
        lidar_tracker,
        lidar_tracker_viz,
        fusion_3D,
        fusion_viz,
        rosbag,
        rviz_node
    ]

    return LaunchDescription(launch_list)
