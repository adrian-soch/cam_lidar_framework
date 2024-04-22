from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():

    transform_explorer = Node(
        package='trad_lidar_detector',
        executable='transform_explorer_node',
        name='transform_explorer_node',
        output='screen'
    )

    rqt_gui = Node(
        package='rqt_gui',
        executable='rqt_gui',
        name='rtq_gui',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
    )

    # Items above will only be launched if they are present in this list
    return LaunchDescription([
        transform_explorer,
        rqt_gui,
        rviz_node
    ])
