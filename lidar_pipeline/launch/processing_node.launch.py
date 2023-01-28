from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lidar_pipeline',
            executable='perception_node',
            name='perception_node',
            output='screen',
            # prefix='valgrind --leak-check=yes ',
            parameters=[
                {"cloud_topic": "/points"},
                {"world_frame": "map"},
                {"camera_frame": "laser_data_frame"},
                {"voxel_leaf_size": 0.25}, # mm
                {"x_filter_min": 2.0},    # mm
                {"x_filter_max": 120.0},     # mm
                {"y_filter_min": -20.0},    # mm
                {"y_filter_max": 10.0},     # mm
                {"z_filter_min": -12.0},    # mm
                {"z_filter_max": 12.0},     # mm
                {"plane_max_iterations": 120},
                {"plane_distance_threshold": 0.4},
                {"cluster_tolerance": 0.6},
                {"cluster_min_size": 4},
                {"cluster_max_size": 2000}
            ]
        )
     ])
