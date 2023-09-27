import os

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

lidar_pipeline_share_dir = get_package_share_directory('lidar_pipeline')
pipeline_params = os.path.join(
    lidar_pipeline_share_dir, 'configs', 'lidar_pipeline_config.yaml')

# Used to change playback rate of ros bag
# A9 data bags were recoreded at 2.5Hz so they need a x4 speedup
# if left as 1 then thre is no rate change
BAG_PLAY_RATE = 1
FLIP_IMAGE = False

'''
@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
              CHANGE THESE PARAMS
@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

Use `BAG_SELECTOR` to pick the desired bag + config to run the pipeline

Note: -1 will use the LiDAR + Webcam with live data
'''
ABS_PATH_TO_ROSBAGS = '/home/adrian/dev/bags/'
BAG_SELECTOR = 2

# Because of the yolov5 includes, its easier to just run this directly
# in the terminal instead of a traditional node
ABS_PATH_TO_FUSION_ENGINE = '/home/adrian/dev/ros2_ws/src/cam_lidar_tools/fusion_engine/fusion_engine'

'''
@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
'''
# MARC Rooftop data without data syncronization
# Fusion and projection will not work
if BAG_SELECTOR == 0:
    FLIP_IMAGE = True
    BAG_NAME = 'dec7_2022/roofTestDark_1_HD_qosOverrride_true/'
    CONFIG_NAME = 'dec7_config.yaml'
elif BAG_SELECTOR == 1:
    FLIP_IMAGE = True
    BAG_NAME = 'dec7_2022/roofTestDaylight_2_FHD_qosOverrride_true/'
    CONFIG_NAME = 'dec7_config.yaml'

# MARC Rooftop data with syncronized lidar + camera
elif BAG_SELECTOR == 2:
    FLIP_IMAGE = True
    BAG_NAME = 'may10_2023/q6_2_may10_2023'
    CONFIG_NAME = 'may10_config.yaml'
elif BAG_SELECTOR == 3:
    FLIP_IMAGE = True
    BAG_NAME = 'may10_2023/q7_2_may10_2023'
    CONFIG_NAME = 'may10_config.yaml'

# A9 data
elif BAG_SELECTOR == 4:
    # Nighttime + rain
    BAG_PLAY_RATE = 4
    BAG_NAME = '2023-09-19_14-49-25_a9_dataset_r02_s04_camSouth2_LidarSouth'
    CONFIG_NAME = 'r02_s04_cam2South_lidarSouth_config.yaml'
elif BAG_SELECTOR == 5:
    # Day time
    BAG_PLAY_RATE = 4
    BAG_NAME = '2023-08-30_13-58-46_a9_dataset_r02_s03_camSouth1_LidarSouth'
    CONFIG_NAME = 'r02_s03_cam1South_lidarSouth_config.yaml'

data_dependant_params = os.path.join(
    lidar_pipeline_share_dir, 'configs', CONFIG_NAME)


def generate_launch_description():

    perception_node = Node(
        package='lidar_pipeline',
        executable='perception_node',
        name='perception_node',
        output='screen',
        parameters=[
            data_dependant_params,
            pipeline_params,
        ]
    )

    lidar2image_node = Node(
        package='lidar_pipeline',
        executable='projection_node',
        name='projection_node',
        output='screen',
        parameters=[
            data_dependant_params,
        ]
    )

    execute_camera_processor = ExecuteProcess(
        cmd=['python3 ./camera_processing_node.py' +
             ' --ros-args -p flip_image:=' + str(FLIP_IMAGE)],
        cwd=[ABS_PATH_TO_FUSION_ENGINE],
        shell=True,
        name='camera_processor',
        emulate_tty=True
    )

    # --- UNUSED CURRENTLY ---
    # s_transform = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',

    #     # params from visual inspection
    #     # To make the road paralell with the XY plane/rviz2 grid
    #     arguments=['0', '0', '0', '0', '0.2', '0', 'map', 'laser_data_frame']
    # )

    # --- UNUSED CURRENTLY ---
    # s_transform2 = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',

    #     # params from visual inspection
    #     # To make the road paralell with the XY plane/rviz2 grid
    #     arguments=['0', '0', '0', '3.1416', '0',
    #                '0', 'map', 'laser_sensor_frame']
    # )

    lidar_classifier = Node(
        package='obj_classifier',
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

    fusion_2D = Node(
        package='fusion_module',
        executable='fusion_node',
        name='fusion_2D_node'
    )

    fusion_viz = Node(
        package='fusion_module',
        executable='fusion_viz_node',
        name='fusion_viz_node',
        parameters=[
            {'flip_image': str(FLIP_IMAGE)},
        ]
    )

    lidar_tracker_viz = Node(
        package='obj_tracker',
        executable='tracker_bbox_viz',
        name='tracker_bbox_viz',
        output='screen',
    )

    if BAG_SELECTOR != -1:
        data_source = ExecuteProcess(
            cmd=[[
                'ros2 bag play ',
                ABS_PATH_TO_ROSBAGS,
                BAG_NAME,
                ' -l',
                ' -r ' + str(BAG_PLAY_RATE)
            ]],
            shell=True
        )
    else:
        data_source = IncludeLaunchDescription(PythonLaunchDescriptionSource(
            [os.path.join(FindPackageShare("fusion_engine"), 'lidar_camera_launch.py')]))

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
        lidar_tracker,
        lidar_classifier,
        lidar2image_node,
        perception_node,
        execute_camera_processor,
        fusion_2D,
        fusion_viz,
        lidar_tracker_viz,
        data_source,
        rviz_node
    ])
