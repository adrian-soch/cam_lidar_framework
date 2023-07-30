#! /usr/bin/env python3
"""
@file lidar_visualization_sync_node.py

@brief This node subsribes to point clouds and images.
    The syncronized callback requires images and pointclopuds to have
    timestamps within 100ms of each other
    Both topics must have the same QoS as the message filter.

@section Author(s)
- Created by Adrian Sochaniwsky on 8/12/2022
"""

import rclpy
from rclpy import qos
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2

import message_filters as mf
from cv_bridge import CvBridge

import ros2_numpy
from lidar_processing.utils.projection_utils import *

import cv2
import random
from matplotlib import pyplot as plt
from numpy.lib.recfunctions import structured_to_unstructured, unstructured_to_structured

CAM_CAL_PATH = '/home/adrian/dev/ros2_ws/src/cam_lidar_tools/fusion_engine/config/1920x1080_ost.yaml'
TRANSFORM_PATH = '/home/adrian/dev/ros2_ws/src/cam_lidar_tools/fusion_engine/config/transforms.yaml'
MULTI_THREAD_EXEC = False  # Set to true to use multithreaded executor
USE_SENSOR_QOS = False  # Set to true to use sensor QoS (ie not rosbag)
PUB_PC2 = True
PUB_IMG = True
SHOW_IMG = False
INCLUDES_INTENSITY = False
IS_FLIPPED = False
RANDOM_COLOUR = False


class LidarNode(Node):
    """
    Create a LidarNode class, which is a subclass of the Node class.
    """

    def __init__(self):
        super().__init__('lidar_visualizer')

        img_topic = 'image_proc/result'
        pc_topic = 'lidar_proc/projected_dets_debug'

        self.camera_mat = get_calib_from_file(
            CAM_CAL_PATH, 'camera_matrix', intrinsic=True)
        transforms = ['lidar2cam_extrinsic', 'lidarData2lidarSensor_extrinsic']
        [self.l2c_mat, self.ld2ls_mat] = get_calib_from_file(
            TRANSFORM_PATH, transforms)

        # Must be 4x4
        self.ld2ls_mat = np.vstack([self.ld2ls_mat, np.array([0, 0, 0, 1])])

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

        self.cmap = plt.cm.get_cmap('hsv', 256)
        self.cmap = np.array([self.cmap(i) for i in range(256)])[:, :3] * 255

        if USE_SENSOR_QOS:
            self.img = mf.Subscriber(
                self, Image, img_topic, qos_profile=qos.qos_profile_sensor_data)
            self.pc2 = mf.Subscriber(
                self, PointCloud2, pc_topic, qos_profile=qos.qos_profile_sensor_data)
        else:
            self.img = mf.Subscriber(self, Image, img_topic)
            self.pc2 = mf.Subscriber(self, PointCloud2, pc_topic)

        # Wait 0.1 seconds, and store a queue up to 6 while trying to match msgs
        self.ts = mf.ApproximateTimeSynchronizer(
            [self.img, self.pc2], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.callback)

        self.get_logger().info('Image and Lidar subscribers started.')
        if PUB_PC2:
            # Publish modified pointcloud
            self.pc2_publisher_ = self.create_publisher(
                PointCloud2, 'points2', qos.qos_profile_sensor_data)
        if PUB_IMG:
            self.img_publisher_ = self.create_publisher(
                Image, 'image2', qos.qos_profile_sensor_data)

    def callback(self, img_msg, pc2_msg):

        self.get_logger().info('Got pc2 and image pair.')
        pc = ros2_numpy.numpify(pc2_msg)

        # Read in the pointcloud, keeping only some fields
        pc = pc.reshape(pc.size)
        if INCLUDES_INTENSITY:
            pc = pc[['x', 'y', 'z', 'intensity']]
        else:
            pc = pc[['x', 'y', 'z']]

        # Convert to unstructured so we can operate easier
        pc = structured_to_unstructured(pc)

        # Remove points behind lidar (in lidar frame this is x-axis)
        # This also removes (0,0,0) points, because they start with x=-0.0
        pc = pc[np.logical_not(pc[:, 0] <= 0)]

        img = self.br.imgmsg_to_cv2(img_msg)

        if IS_FLIPPED:
            img = cv2.flip(img, -1)

        '''
        Transform pc
        ###################################################
        '''

        # Return x,y,z,1
        xyz1 = np.hstack(
            [pc[:, :3], np.ones((pc.shape[0], 1), dtype=np.float32)])

        # from dataFrame -> sensorFrame -> cameraFrame
        #       (3*4) @ (4*4) @ (4*N) = (3*N)
        temp = self.camera_mat @ self.l2c_mat @  self.ld2ls_mat @ xyz1.T
        print(self.camera_mat @ self.l2c_mat @  self.ld2ls_mat)

        '''
        Project pc to img
        ###################################################
        '''
        # Get projection on image plane
        # (3*3) @ (3*N) = (3*N)
        px_proj = (temp)[0]
        px_proj = (px_proj[:2, :] / px_proj[2:]).T

        # Restrict points to camera fov
        depth = pc[:, 0]
        depth = depth[np.logical_not(np.logical_or(
            px_proj[:, 0] < 0, px_proj[:, 0] > img.shape[1]))]
        px_proj = px_proj[np.logical_not(np.logical_or(
            px_proj[:, 0] < 0, px_proj[:, 0] > img.shape[1]))]

        depth = depth[np.logical_not(np.logical_or(
            px_proj[:, 1] < 0, px_proj[:, 1] > img.shape[0]))]
        px_proj = px_proj[np.logical_not(np.logical_or(
            px_proj[:, 1] < 0, px_proj[:, 1] > img.shape[0]))]

        if RANDOM_COLOUR:
            for idx, i in enumerate(px_proj):
                color = (1, 2, 3)
                cv2.circle(img, np.int32(i), 5, color, -1)
        else:
            depth_max = np.max(depth)
            for idx, i in enumerate(px_proj):
                color = int((depth[idx]/(depth_max))*255)
                color = self.cmap[np.clip(color, a_min=0, a_max=255), :]
                cv2.circle(img, np.int32(i), 5, color, -1)

        if SHOW_IMG:
            cv2.imshow('proj', img)
            cv2.waitKey(1)

        if PUB_PC2:
            '''
            Publish results
            ###################################################
            '''
            # Restructure so we can convert back to ros2 pc2
            temp = temp.T
            if INCLUDES_INTENSITY:
                # Put intensity back
                temp = np.hstack([temp, pc[:, 3:]])

            else:
                temp = np.hstack(
                    [pc[:, :3], np.ones((pc.shape[0], 1), dtype=np.float32)])

            # Restructure so we can convert back to ros2 pc2
            pc = unstructured_to_structured(temp, dtype=np.dtype(
                [('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('intensity', '<f4')]))

            msg2 = ros2_numpy.msgify(PointCloud2, pc)
            # Set the same header as original msg
            msg2.header = pc2_msg.header
            self.pc2_publisher_.publish(msg2)
        if PUB_IMG:
            msg3 = self.br.cv2_to_imgmsg(img)
            msg3.header = img_msg.header
            self.img_publisher_.publish(msg3)


def main(args=None):
    rclpy.init(args=args)

    lidar_visualizer = LidarNode()

    if MULTI_THREAD_EXEC:
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(lidar_visualizer)
        executor.spin()
    else:
        rclpy.spin(lidar_visualizer)

    # Destroy the node explicitly
    lidar_visualizer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
