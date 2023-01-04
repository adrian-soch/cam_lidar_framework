#! /usr/bin/env python3
"""
@file lidar_visualization_node.py

@brief This node subsribes to point clouds and projects the
    pointcloud onto the image. There is no sync/aligment
    of data, this method is naive.

@section Author(s)
- Created by Adrian Sochaniwsky on 5/12/2022
"""

import rclpy
from rclpy import qos
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

import ros2_numpy
from lidar_processing.utils.projection_utils import get_calib_from_file

import cv2
import time
import numpy as np
import threading, queue
from matplotlib import pyplot as plt
from numpy.lib.recfunctions import structured_to_unstructured, unstructured_to_structured

'''
Constants
###################################################
'''
CAM_CAL_PATH = '/home/adrian/dev/ros2_ws/src/fusion_engine/config/1280x720_ost.yaml'
TRANSFORM_PATH = '/home/adrian/dev/ros2_ws/src/fusion_engine/config/transforms.yaml'
MULTI_THREAD_EXEC = False # Set to true to use multithreaded executor
USE_SENSOR_QOS = True # Set to true to use sensor QoS (ie not rosbag)
PUB_PC2 = False
PUB_IMG = True
SHOW_IMG = False

class LidarNode(Node):
    """
    Create a LidarNode class, which is a subclass of the Node class.
    """
    def __init__(self):
        super().__init__('lidar_visualizer')

        self.camera_mat, self.dist = get_calib_from_file(CAM_CAL_PATH, ['camera_matrix', 'distortion_coefficients'], intrinsic=True)
        transforms = ['lidar2cam_extrinsic', 'lidarData2lidarSensor_extrinsic']
        [self.l2c_mat, self.ld2ls_mat] = get_calib_from_file(TRANSFORM_PATH, transforms)

        # Must be 4x4
        self.ld2ls_mat = np.vstack([self.ld2ls_mat, np.array([0,0,0,1], dtype=np.float32)])

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

        if USE_SENSOR_QOS:
            QOS = qos.qos_profile_sensor_data
        else:
            QOS = 5

        callback_group = ReentrantCallbackGroup()
        self.subscription = self.create_subscription(
            PointCloud2,'points',
            self.lidar_callback,QOS,
            callback_group=callback_group)
        self.subscription  # prevent unused variable warning

        self.im_subscription = self.create_subscription(
            Image,'image',
            self.image_callback,QOS,
            callback_group=callback_group)
        self.im_subscription  # prevent unused variable warning

        self.PC_Queue = queue.SimpleQueue()
        self.IMG_Queue = queue.SimpleQueue()

        self.cmap = plt.cm.get_cmap('hsv', 256)
        self.cmap = np.array([self.cmap(i) for i in range(256)])[:, :3] * 255

        self.get_logger().info('Image and Lidar subscribers started.')

        if PUB_PC2:
            # Publish modified pointcloud
            self.pc2_publisher_ = self.create_publisher(PointCloud2, 'points2', qos.qos_profile_sensor_data)
        if PUB_IMG:
            self.img_publisher_ = self.create_publisher(Image, 'image2', qos.qos_profile_sensor_data)


    def lidar_callback(self, msg):
        t1 = time.clock_gettime(time.CLOCK_THREAD_CPUTIME_ID)

        pc = ros2_numpy.numpify(msg)

        # Only retain x, y, z, intensity
        pc = pc.reshape(pc.size)
        pc = pc[['x','y','z', 'intensity']]

        # Convert to unstructured so we can operate easier
        pc = structured_to_unstructured(pc)

        # Remove points behind lidar (in lidar frame this is x-axis)
        # This also removes (0,0,0) points, because they start with x=-0.0
        pc = pc[np.logical_not(pc[:,0] <= 0)]

        self.PC_Queue.put([pc, msg.header])

        t2 = time.clock_gettime(time.CLOCK_THREAD_CPUTIME_ID)
        self.get_logger().info('PC added to queue: '+ str(t2-t1))

    def image_callback(self, msg):
        # Get image (img)

        t1 = time.clock_gettime(time.CLOCK_THREAD_CPUTIME_ID)
        img = ros2_numpy.numpify(msg)
        self.IMG_Queue.put([img, msg.header])

        t2 = time.clock_gettime(time.CLOCK_THREAD_CPUTIME_ID)
        self.get_logger().info('IMG added to queue: '+ str(t2-t1))

    def worker(self):
        while True:
            t1 = time.clock_gettime(time.CLOCK_THREAD_CPUTIME_ID)
            
            # Take pc from queue
            if self.PC_Queue.empty() or self.IMG_Queue.empty():
                print('Queue empty')
                time.sleep(0.005)
                continue

            [pc, pc_header] = self.PC_Queue.get()
            [img, img_header] = self.IMG_Queue.get()

            '''
            Process img
            ###################################################
            '''
            img = cv2.flip(img, -1)
            newcameramtx, roi = cv2.getOptimalNewCameraMatrix(self.camera_mat, self.dist, (img.shape[1],img.shape[0]), 1, (img.shape[1],img.shape[0]))
            img = cv2.undistort(img, self.camera_mat, self.dist, newcameramtx)

            t2 = time.clock_gettime(time.CLOCK_THREAD_CPUTIME_ID)

            '''
            Transform pc
            ###################################################
            '''

            # Return x,y,z,1
            xyz1 = np.hstack([pc[:, :3], np.ones((pc.shape[0], 1), dtype=np.float32)])

            # from dataFrame -> sensorFrame -> cameraFrame
            #       (3*4) @ (4*4) @ (4*N) = (3*N)
            temp = self.l2c_mat @  self.ld2ls_mat @ xyz1.T
            
            '''
            Project pc to img
            ###################################################
            '''
            # Get projection on image plane
            # (3*3) @ (3*N) = (3*N)
            px_proj = (self.camera_mat @ temp)

            # Get depth
            depth = px_proj[2 :].T

            # Divide by depth to get 2-D projection
            px_proj = (px_proj[:2,:] / px_proj[2 :]).T

            # Restrict points to camera fov
            depth = depth[np.logical_not(np.logical_or(px_proj[:,0] < 0, px_proj[:,0] > img.shape[1]))]
            px_proj = px_proj[np.logical_not(np.logical_or(px_proj[:,0] < 0, px_proj[:,0] > img.shape[1]))]

            depth = depth[np.logical_not(np.logical_or(px_proj[:,1] < 0, px_proj[:,1] > img.shape[0]))]
            px_proj = px_proj[np.logical_not(np.logical_or(px_proj[:,1] < 0, px_proj[:,1] > img.shape[0]))]  


            t3 = time.clock_gettime(time.CLOCK_THREAD_CPUTIME_ID)

            norm = plt.Normalize()
            colors = plt.cm.rainbow(norm(depth), bytes=True)
            colors = colors[:,0,0:3]

            for idx,i in enumerate(px_proj):
                c = tuple(map(int, colors[idx]))
                cv2.circle(img, np.int32(i), 2, c, -1)

            if SHOW_IMG:
                cv2.imshow('proj', img)
                cv2.waitKey(1)
            
            t4 = time.clock_gettime(time.CLOCK_THREAD_CPUTIME_ID)

            '''
            Publish results
            ###################################################
            '''
            if PUB_PC2:
                # Put intensity back
                temp = temp.T
                temp = np.hstack([temp, pc[:, 3:]])

                # Restructure so we can convert back to ros2 pc2
                pc = unstructured_to_structured(temp, dtype=np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('intensity', '<f4')]))
                msg2 = ros2_numpy.msgify(PointCloud2, pc)
                
                # Set the same header as original msg
                msg2.header = pc_header
                self.pc2_publisher_.publish(msg2)

            if PUB_IMG:
                msg3 = self.br.cv2_to_imgmsg(img)
                msg3.header = img_header
                self.img_publisher_.publish(msg3)

            t5 = time.clock_gettime(time.CLOCK_THREAD_CPUTIME_ID)
            # Print the time it took in seconds
            self.get_logger().info('Q '+str(t2-t1)+' PC '+str(t3-t2)+' Proj '+str(t4-t3)+' Pub '+str(t5-t4))

def main(args=None):
    rclpy.init(args=args)

    lidar_visualizer = LidarNode()

    x = threading.Thread(target=lidar_visualizer.worker, daemon=True)
    x.start()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(lidar_visualizer)
    executor.spin()

    # Destroy the node explicitly
    lidar_visualizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()