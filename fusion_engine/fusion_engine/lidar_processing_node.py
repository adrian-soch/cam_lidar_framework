#! /usr/bin/env python3
"""
@file lidar_processing_node.py
@brief This node subsribes to point clouds and runs detection+tracking
@section Author(s)
- Created by Adrian Sochaniwsky on 03/01/2023
"""

import rclpy
from rclpy import qos
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

import ros2_numpy

import cv2
import numpy as np
from numpy.lib.recfunctions import structured_to_unstructured, unstructured_to_structured

USE_SENSOR_SUB_QOS = False
USE_SENSOR_PUB_QOS = False
FRONT_VIEW_ONLY = False


class LidarSubscriber(Node):
    """
    Create a LidarSubscriber class, which is a subclass of the Node class.
    """

    def __init__(self):
        super().__init__('lidar_processor')

        if USE_SENSOR_SUB_QOS:
            QOS_S = qos.qos_profile_sensor_data
        else:
            QOS_S = 5

        self.subscription = self.create_subscription(
            PointCloud2,
            'points',
            self.lidar_callback,
            QOS_S)
        self.subscription  # prevent unused variable warning

        if USE_SENSOR_PUB_QOS:
            QOS_P = qos.qos_profile_sensor_data
        else:
            QOS_P = 5

        self.pc2_publisher_ = self.create_publisher(
            PointCloud2, 'points2', QOS_P)

        self.get_logger().info('Subscriber and Publisher started.')

    def lidar_callback(self, msg):

        pc = ros2_numpy.numpify(msg)

        # Only retain x, y, z, intensity
        pc = pc.reshape(pc.size)
        pc = pc[['x', 'y', 'z', 'intensity']]

        # Convert to unstructured so we can operate easier
        pc = structured_to_unstructured(pc)

        if FRONT_VIEW_ONLY:
            # Remove points behind lidar (in lidar frame this is x-axis)
            # This also removes (0,0,0) points, because they start with x=-0.0
            pc = pc[np.logical_not(pc[:, 0] <= 0)]

        # Voxel filter to remove # of points?

        # Remove large planes

        # Background point removal

        # Clustering

        '''
        Publish results
        ###################################################
        '''
        # Restructure so we can convert back to ros2 pc2
        pc = unstructured_to_structured(pc, dtype=np.dtype(
            [('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('intensity', '<f4')]))
        msg2 = ros2_numpy.msgify(PointCloud2, pc)

        # Set the same header as original msg
        msg2.header = msg.header
        self.pc2_publisher_.publish(msg2)


def main(args=None):
    rclpy.init(args=args)

    lidar_processor = LidarSubscriber()

    rclpy.spin(lidar_processor)

    # Destroy the node explicitly
    lidar_processor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
