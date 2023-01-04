#! /usr/bin/env python3
"""
@file lidar_processing_node.py
@brief This node subsribes to point clouds and runs detection+tracking
@section Author(s)
- Created by Adrian Sochaniwsky on 03/01/2023
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


class RadarSubscriber(Node):
    """
    Create a LidarSubscriber class, which is a subclass of the Node class.
    """
    def __init__(self):
        super().__init__('lidar_processor')
        self.subscription = self.create_subscription(
            PointCloud2,
            'Radar',
            self.radar_callback,
            5)
        self.subscription  # prevent unused variable warning

    def radar_callback(self, msg):
        self.get_logger().info('Point cloud received.')


def main(args=None):
    rclpy.init(args=args)

    radar_processor = RadarSubscriber()

    rclpy.spin(radar_processor)

    # Destroy the node explicitly
    radar_processor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()