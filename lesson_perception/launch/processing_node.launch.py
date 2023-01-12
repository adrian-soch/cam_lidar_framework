from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lesson_perception',
            executable='perception_node',
            name='perception_node',
            output='screen',
            parameters=[
                {"cloud_topic": "/points"},
                {"world_frame": "laser_data_frame"},
                {"camera_frame": "laser_data_frame"},
                {"voxel_leaf_size": 10.0}, # mm
                {"x_filter_min": 0.0},    # mm
                {"x_filter_max": 50.0},     # mm
                {"y_filter_min": -10.0},    # mm
                {"y_filter_max": 10.0},     # mm
                {"z_filter_min": -20.0},    # mm
                {"z_filter_max": 10.0},     # mm
                {"plane_max_iterations": 100},
                {"plane_distance_threshold": 0.03},
                {"cluster_tolerance": 0.02},
                {"cluster_min_size": 1},
                {"cluster_max_size": 10000}
            ]
        )
     ])
