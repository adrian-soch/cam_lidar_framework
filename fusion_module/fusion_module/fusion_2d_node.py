#! /usr/bin/env python3
"""
@file fusion_2d_image_coords.py

@brief This node subscribes to 2D detections/tracks from camera and lidar sensors
    It will associate dets based on IoU in the image plane
    Unmatched dets + matched dets list will be tracked via SORT MOT tracker
    Final results will be published as a Detection2DArray

@section Author(s)
- Created by Adrian Sochaniwsky on 25/09/2023
"""

import os
os.environ["OMP_NUM_THREADS"] = "1" 
os.environ["OPENBLAS_NUM_THREADS"] = "1"
os.environ["MKL_NUM_THREADS"] = "1"
os.environ["VECLIB_MAXIMUM_THREADS"] = "1"
os.environ["NUMEXPR_NUM_THREADS"] = "1" 

import numpy as np
import time

import rclpy
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray

from fusion_module.sort import Sort, iou_batch, linear_assignment
from fusion_module.utils import createDetection2DArr, detection2DArray2Numpy

class DetectionSyncNode(Node):
    def __init__(self):
        super().__init__('detection_sync_node')

        # Get the topic name from the ROS parameter server
        self.world_frame = self.declare_parameter(
            'world_frame', 'map').get_parameter_value().string_value

        cam_track_topic = self.declare_parameter(
            'cam_track_topic', '/image_proc/tracks').get_parameter_value().string_value
        lidar2d_track_topic = self.declare_parameter(
            'lidar2d_track_topic', 'image_proc/lidar_track_2D').get_parameter_value().string_value

        # Create subscribers and the approximate syncronizer message filter
        cam_sub = Subscriber(self, Detection2DArray, cam_track_topic)
        lidar_sub = Subscriber(self, Detection2DArray, lidar2d_track_topic)
        sync = ApproximateTimeSynchronizer(
            [cam_sub, lidar_sub], queue_size=3, slop=0.65)
        sync.registerCallback(self.callback)

        # Create SORT instance for fused detections
        self.tracker = Sort(max_age=4, min_hits=3, iou_threshold=0.01)

        # Create publisher
        self.track_publisher_ = self.create_publisher(
            Detection2DArray, 'image_proc/fusion_tracks', 2)
        
        self.get_logger().info('Fusion Module initialized.')

    def callback(self, cam_2d_dets: Detection2DArray, lidar_2d_dets: Detection2DArray):
        """This callback will take the 2 sensors detection arrays and produce a final fused and tracked array

        Args:
            cam_2d_dets (Detection2DArray): Cam detections/tracks
            lidar_2d_dets (Detection2DArray): LiDAR detections/tracks
        """
        t1 = time.clock_gettime(time.CLOCK_THREAD_CPUTIME_ID)

        # Convert message arrays to numpy
        cam_dets = detection2DArray2Numpy(cam_2d_dets.detections, sensor_type='C')
        lidar_dets = detection2DArray2Numpy(lidar_2d_dets.detections, sensor_type='L')

        # Simple fusion with IoU based association
        fused_detections = self.fuse(
            cam_arr=cam_dets, lidar_arr=lidar_dets, iou_threshold=0.1)

        # Update SORT with detections
        track_ids = self.tracker.update(fused_detections)

        # Create and Publish 2D Detections with Track IDs
        track_msg_arr = createDetection2DArr(
            track_ids, cam_2d_dets.header)
        self.track_publisher_.publish(track_msg_arr)

        # Get and publish the execution time
        t2 = time.clock_gettime(time.CLOCK_THREAD_CPUTIME_ID)
        self.get_logger().info('Tracked {:4d} objects in {:.1f} msec.'.format(
            len(track_msg_arr.detections), (t2-t1)*1000))

    def fuse(self, cam_arr, lidar_arr, iou_threshold=0.3) -> np.ndarray:
        """Perform late fusion between 2 sets of Axis-alogned bounding boxes from 2 different sensors

        Associate the 2 lists. For all matched detections just keep the camera detection.
        Keep all unmatched detections. This will use all camera detections
        including the longer distance and also use lidar detections in
        poor conditions the camera didnt get

        NOTE code is adapted from SORT algorithm

        Args:
            cam_arr (np.ndarray): Cam detection array
            lidar_arr (np.ndarray): Lidar detection array
            iou_threshold (float, optional): Threshold for IoU based association. Defaults to 0.3.

        Returns:
            np.ndarray: list of fused filt_match_inds and unmatched detection from both sensors
        """

        # Get NxN matrix of IoU between detections
        iou_matrix = iou_batch(cam_arr, lidar_arr)

        # Find all filt_match_inds
        if min(iou_matrix.shape) > 0:
            a = (iou_matrix > iou_threshold).astype(np.int32)
            if a.sum(1).max() == 1 and a.sum(0).max() == 1:
                matched_indices = np.stack(np.where(a), axis=1)
            else:
                matched_indices = linear_assignment(-iou_matrix)
        else:
            matched_indices = np.zeros(shape=(0, 2))

        # Get unmached camera detections
        unmatched_cam_dets_inds = []
        for d in range(len(cam_arr)):
            if (d not in matched_indices[:, 0]):
                unmatched_cam_dets_inds.append(d)

        # Get unmatched lidar detections
        unmatched_lidar_dets_inds = []
        for d in range(len(lidar_arr)):
            if (d not in matched_indices[:, 1]):
                unmatched_lidar_dets_inds.append(d)

        # Filter out matched with low IOU
        filt_match_inds = []
        for m in matched_indices:
            if (iou_matrix[m[0], m[1]] < iou_threshold):
                unmatched_cam_dets_inds.append(m[0])
                unmatched_lidar_dets_inds.append(m[1])
            else:
                filt_match_inds.append(m.reshape(1, 2))
        if (len(filt_match_inds) == 0):
            filt_match_inds = np.empty((0, 2), dtype=int)
        else:
            filt_match_inds = np.concatenate(filt_match_inds, axis=0)

        fused_dets = []
        for i in filt_match_inds:
            # Change sensor type to 'B' (both)
            det = cam_arr[i[0], :]
            det[-1] = float(ord('B'))
            fused_dets.append(cam_arr[i[0], :])

        unmatched_cam_dets = []
        for i in unmatched_cam_dets_inds:
            unmatched_cam_dets.append(cam_arr[i, :])

        unmatched_lidar_dets = []
        for i in unmatched_lidar_dets_inds:
            unmatched_lidar_dets.append(lidar_arr[i, :])

        return np.array(fused_dets + unmatched_cam_dets + unmatched_lidar_dets)


def main():
    rclpy.init()
    node = DetectionSyncNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
