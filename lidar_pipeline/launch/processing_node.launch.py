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
                {"voxel_leaf_size": 0.2}, # All in meters
                {"plane_max_iterations": 120},
                {"plane_distance_threshold": 0.35},
                {"cluster_tolerance": 1.35},
                {"cluster_min_size": 2},
                {"cluster_max_size": 2000}
            ]
        )
     ])
