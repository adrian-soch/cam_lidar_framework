from launch import LaunchDescription
from launch_ros.actions import Node

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
            arguments = ['0', '0', '0', '3.1416', '0.2', '0', 'map', 'laser_sensor_frame']
        )

    lidar_tracker = Node(
            package='obj_tracker',
            executable='obj_tracker',
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
            {"x_filter_max": 120.0},
            {"y_filter_min": -25.0},
            {"y_filter_max": 10.0},
            {"z_filter_min": -15.0},
            {"z_filter_max": 15.0},
            {"plane_max_iterations": 100},
            {"plane_distance_threshold": 0.4},
            {"cluster_tolerance": 1.5},
            {"cluster_min_size": 3},
            {"cluster_max_size": 2000}
        ]
    )
    
    # Items above will only be launched if they are present in this list
    return LaunchDescription([
        s_transform2,
        lidar_tracker,
        perception_node,
        s_transform
   ])