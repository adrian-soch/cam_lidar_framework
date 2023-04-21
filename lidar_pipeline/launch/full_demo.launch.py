from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare

# from ament_index_python.packages import get_package_share_directory

ABS_PATH_TO_ROSBAGS = '/home/adrian/dev/bags'

def generate_launch_description():

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

    perception_node = Node(
        package='lidar_pipeline',
        executable='perception_node',
        name='perception_node',
        # prefix='valgrind --leak-check=yes ',
        output='screen',
        parameters=[
            {"cloud_topic": "/points"},
            {"world_frame": "map"},
            {"camera_frame": "laser_data_frame"},
            {"voxel_leaf_size": 0.25}, # All in meters
            {"x_filter_min": 1.0},
            {"x_filter_max": 80.0},
            {"y_filter_min": -16.0},
            {"y_filter_max": 5.0},
            {"z_filter_min": -10.0},
            {"z_filter_max": 15.0},
            # {"x_filter_min": 0.5},
            # {"x_filter_max": 120.0},
            # {"y_filter_min": -18.0},
            # {"y_filter_max": 20.0},
            # {"z_filter_min": -10.0},
            # {"z_filter_max": 15.0},
            {"plane_max_iterations": 120},
            {"plane_distance_threshold": 0.4},
            {"cluster_tolerance": 1.5},
            {"cluster_min_size": 3},
            {"cluster_max_size": 2000}
        ]
    )

    rosbag_play = ExecuteProcess(
        cmd=[[
            'ros2 bag play ',
            ABS_PATH_TO_ROSBAGS,
            '/dec7_2022/roofTestDark_1_HD_qosOverrride_true/',
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
        s_transform,
        rosbag_play,
        rviz_node
   ])