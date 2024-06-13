import os

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

lidar_pipeline_share_dir = get_package_share_directory('trad_lidar_detector')
pipeline_params = os.path.join(
    lidar_pipeline_share_dir, 'configs', 'lidar_pipeline_config.yaml')

rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('cam_lidar_bringup'), 'configs', 'rviz.rviz']
    )

# Used to change playback rate of ros bag
# A9 data bags were recoreded at 2.5Hz so they need a x4 speedup
# if left as 1 then thre is no rate change
BAG_PLAY_RATE = 1.0
FLIP_IMAGE = False

BAG_PLAY_LOOP = True

'''
@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
              CHANGE THESE PARAMS
@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

Use `BAG_SELECTOR` to pick the desired bag + config to run the pipeline
Note: -1 will use the LiDAR + Webcam with live data
'''
ABS_PATH_TO_ROSBAGS = '/home/adrian/dev/bags/'

# 10, 7, 6, 12, 13
BAG_SELECTOR = 7

# Determines what kind of output you want, Video/Rviz2/csv_tracker_data
SAVE_OUTPUT_VIDEO = False
SAVE_CSV_FUSION_OUTPUT = False
SHOW_RVIZ = True

# Fusion Overrides
LIDAR_RESULT_ONLY = False
CAM_RESULT_ONLY = False

# Path to YOLOv5, YOLOv8, or YOLOv9 model [.pt or .engine]
MODEL_NAME = 'yolov8m.pt'
MODEL_PATH = os.path.join(get_package_share_directory('camera_det2d'), MODEL_NAME)

# Enable camera 3d detections
ENABLE_CAM_3D = False
'''
@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
'''
if BAG_SELECTOR == -1:
    # FLIP_IMAGE = True
    CONFIG_NAME = 'default_config.yaml'

elif BAG_SELECTOR == 10:
    # Cleaned bag - mismatched frames removed
    # Good for use with metrics
    BAG_NAME = 'cleaned_bags/dec7_dhd1_clean_short'
    CONFIG_NAME = 'dec7_config.yaml'
elif BAG_SELECTOR == 12:
    BAG_NAME = 'cleaned_bags/may10_r5_clean'
    CONFIG_NAME = 'may10_config.yaml'
elif BAG_SELECTOR == 13:
    BAG_NAME = 'cleaned_bags/oct18_r1_clean'
    CONFIG_NAME = 'oct18_r1_config.yaml'
elif BAG_SELECTOR == 14:
    BAG_NAME = 'cleaned_bags/dec14_2023_ok_clean'
    CONFIG_NAME = 'dec14_config.yaml'

# MARC Rooftop data with syncronized lidar + camera
elif BAG_SELECTOR == 2:
    # WARNING: 11 missing Pointcloud frames!
    FLIP_IMAGE = True
    BAG_NAME = 'may10_2023/q6_2_may10_2023'
    CONFIG_NAME = 'may10_config.yaml'
elif BAG_SELECTOR == 6:
    # Cleaned bag - mismatched frames removed
    # Good for use with metrics
    BAG_NAME = 'cleaned_bags/may10_q7_clean'
    CONFIG_NAME = 'may10_config.yaml'
elif BAG_SELECTOR == 7:
    # Cleaned bag - mismatched frames removed
    # Good for use with metrics
    BAG_NAME = 'cleaned_bags/oct18_r9_clean'
    CONFIG_NAME = 'oct18_config.yaml'

# A9 data
elif BAG_SELECTOR == 8:
    # Nighttime + rain
    BAG_PLAY_RATE = 4
    BAG_NAME = '2023-09-19_14-49-25_a9_dataset_r02_s04_camSouth2_LidarSouth'
    CONFIG_NAME = 'r02_s04_cam2South_lidarSouth_config.yaml'
elif BAG_SELECTOR == 9:
    # Day time
    BAG_PLAY_RATE = 4
    BAG_NAME = '2023-08-30_13-58-46_a9_dataset_r02_s03_camSouth1_LidarSouth'
    CONFIG_NAME = 'r02_s03_cam1South_lidarSouth_config.yaml'
else:
    print('Invalid bag selection!')
    exit(-1)

# Set and validate parameters

if SAVE_CSV_FUSION_OUTPUT:
    BAG_PLAY_RATE = 0.16

if LIDAR_RESULT_ONLY and CAM_RESULT_ONLY:
    print('Error: Must use lidar or camera results for fusion.')
    exit(ValueError)

START_BAG_DELAY = 0.0
if SAVE_OUTPUT_VIDEO:
    START_BAG_DELAY = 7.5

if SAVE_OUTPUT_VIDEO:
    BAG_PLAY_LOOP = False

data_dependant_params = os.path.join(
    lidar_pipeline_share_dir, 'configs', CONFIG_NAME)


def generate_launch_description():

    perception_node = Node(
        package='trad_lidar_detector',
        executable='perception_node',
        name='perception_node',
        output='screen',
        parameters=[
            data_dependant_params,
            pipeline_params,
        ]
    )

    lidar2image_node = Node(
        package='trad_lidar_detector',
        executable='projection_node',
        name='projection_node',
        output='screen',
        parameters=[
            data_dependant_params,
        ]
    )

    camera_det2d = Node(
        package='camera_det2d',
        executable='camera_processor',
        parameters=[
            {'image_topic': 'image'},
            {'detection_topic': 'image_proc/dets'},
            {'out_image_topic': 'image_proc/result'},
            {'confidence': 0.3},
            {'model_path': MODEL_PATH},
            {'flip_image': FLIP_IMAGE}
        ]
    )

    lidar_classifier = Node(
        package='lidar_obj_classifier',
        executable='object_classifier',
        name='lidar_obj_classifier',
        output='screen',
    )

    lidar_tracker = Node(
        package='obj_tracker',
        executable='object_tracker',
        name='lidar_obj_tracker',
        output='screen',
    )

    lidar_tracker_viz = Node(
        package='obj_tracker',
        executable='tracker_bbox_viz',
        name='tracker_bbox_viz',
        output='screen',
    )

    fusion_2D = Node(
        package='fusion_2d',
        executable='fusion_node',
        name='fusion_2D_node',
        parameters=[
            {'lidar_only_override': LIDAR_RESULT_ONLY},
            {'camera_only_override': CAM_RESULT_ONLY}
        ]
    )

    video_name = BAG_NAME.split('/')[-1]
    fusion_viz = Node(
        package='fusion_2d',
        executable='fusion_viz_node',
        name='fusion_viz_node',
        parameters=[
            {'flip_image': FLIP_IMAGE},
            {'save_video': SAVE_OUTPUT_VIDEO},
            {'video_name': video_name},
        ]
    )

    camera_det3D = Node(
            package='camera_det3d',
            executable='camera_det3d',
            name='camera_det3d',
            output='screen',
            parameters=[data_dependant_params],
        )

    save_csv_fusion = Node(
        package='fusion_2d',
        executable='detection2csv_node',
        name='detection2csv_node',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file]
    )

    if BAG_SELECTOR != -1:
        bag_cmd = ['ros2 bag play ',
                   ABS_PATH_TO_ROSBAGS, BAG_NAME,
                   ' -r ' + str(BAG_PLAY_RATE)]

        if BAG_PLAY_LOOP:
            bag_cmd.append(' -l')

        data_source = TimerAction(
            period=START_BAG_DELAY,
            actions=[
                ExecuteProcess(
                    cmd=[bag_cmd],
                    shell=True)
            ],
        )
    else:
        data_source = IncludeLaunchDescription(PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('cam_lidar_bringup'), 'lidar_camera_launch.py'])
        ))

    launch_list = [
        lidar_tracker,
        lidar_classifier,
        lidar2image_node,
        perception_node,
        camera_det2d,
        fusion_2D,
        fusion_viz,
        data_source,
    ]

    if SHOW_RVIZ:
        launch_list.append(rviz_node)
        launch_list.append(lidar_tracker_viz)

    if SAVE_CSV_FUSION_OUTPUT:
        launch_list.append(save_csv_fusion)

    if ENABLE_CAM_3D:
        launch_list.append(camera_det3D)

    # Items above will only be launched if they are present in this list
    return LaunchDescription(launch_list)
