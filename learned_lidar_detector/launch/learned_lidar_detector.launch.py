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

lidar_pipeline_share_dir = get_package_share_directory('lidar_pipeline')
pipeline_params = os.path.join(
    lidar_pipeline_share_dir, 'configs', 'lidar_pipeline_config.yaml')

rviz_config_file = PathJoinSubstitution(
    [FindPackageShare('learned_lidar_detector'), 'rviz.rviz']
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
BAG_SELECTOR = 9

# Determines what kind of output you want, Video/Rviz2/csv_tracker_data
SHOW_RVIZ = True

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


START_BAG_DELAY = 0.0
data_dependant_params = os.path.join(
    lidar_pipeline_share_dir, 'configs', CONFIG_NAME)


def generate_launch_description():

    perception_node = Node(
        package='lidar_pipeline',
        executable='perception_node',
        output='log',
        parameters=[
            data_dependant_params,
            pipeline_params,
        ]
    )

    share_dir = get_package_share_directory('learned_lidar_detector')
    model_path = os.path.join(share_dir, 'best.engine')
    lidar_perception_node = Node(
        package='learned_lidar_detector',
        executable='learned_lidar_detector',
        output='screen',
        parameters=[
            data_dependant_params,
            {'lidar_topic': 'points'},
            {'model_path': model_path}
        ]
    )

    # lidar_tracker = Node(
    #     package='obj_tracker',
    #     executable='object_tracker',
    #     output='screen',
    # )

    lidar_viz = Node(
        package='obj_tracker',
        executable='tracker_bbox_viz',
        output='screen',
        parameters=[
            {'topic_name': 'ld_proc/dets'},
            {'bbox_marker_topic': 'ld_proc/bboxs'},
            {'tracklet_topic': 'ld_proc/tracklets'}
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
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
        # perception_node,
        lidar_perception_node,
        data_source,
    ]

    if SHOW_RVIZ:
        launch_list.append(rviz_node)
        launch_list.append(lidar_viz)

    # Items above will only be launched if they are present in this list
    return LaunchDescription(launch_list)
