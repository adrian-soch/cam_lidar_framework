import os

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

from launch_ros.substitutions import FindPackageShare

'''
Rosbag and param file must match for proper functionality
example: 
    bag: q7_2_may10_2023
    data_dependant_params file: may10_config.yaml
'''

ABS_PATH_TO_ROSBAGS = '/home/adrian/dev/bags/'
# BAG_NAME = 'dec7_2022/roofTestDark_1_HD_qosOverrride_true/'
# BAG_NAME = 'dec7_2022/roofTestDaylight_2_FHD_qosOverrride_true/'
# BAG_NAME = 'may10_2023/q6_2_may10_2023'
BAG_NAME = 'may10_2023/q7_2_may10_2023'

share_dir = get_package_share_directory('lidar_pipeline')
pipeline_params = os.path.join(share_dir, 'configs', 'lidar_pipeline_config.yaml')
data_dependant_params = os.path.join(share_dir, 'configs', 'may10_config.yaml')

def generate_launch_description():

    perception_node = Node(
            package='lidar_pipeline',
            executable='perception_node',
            name='perception_node',
            output='screen',
            parameters=[
                pipeline_params,
                data_dependant_params
            ]
    )

    execute_camera_processor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('fusion_engine'),
            'cam_processor_launch.py')])
        )

    s_transform = Node(
            package='tf2_ros',
            executable='static_transform_publisher',

            #params from visual inspection
            # To make the road paralell with the XY plane/rviz2 grid
            arguments = ['0', '0', '0', '0', '0.2', '0', 'map', 'laser_data_frame']
        )

    s_transform2 = Node(
            package='tf2_ros',
            executable='static_transform_publisher',

            #params from visual inspection
            # To make the road paralell with the XY plane/rviz2 grid
            arguments = ['0', '0', '0', '3.1416', '0', '0', 'map', 'laser_sensor_frame']
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

    rosbag_play = ExecuteProcess(
        cmd=[[
            'ros2 bag play ',
            ABS_PATH_TO_ROSBAGS,
            BAG_NAME,
            ' -l'
        ]],
        shell=True
    )

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
        s_transform2,
        lidar_tracker,
        perception_node,
        execute_camera_processor,
        s_transform,
        lidar_tracker_viz,
        rosbag_play,
        rviz_node
   ])