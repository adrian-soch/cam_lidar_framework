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
from lap import lapjv
import time
import numpy as np

import rclpy
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.duration import Duration
from rclpy.node import Node
from sort import Sort
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import Marker, MarkerArray

from utils import createDetection2DArr, detection2DArray2Numpy


def linear_assignment(cost_matrix):
    _, x, y = lapjv(cost_matrix, extend_cost=True)
    return np.array([[y[i], i] for i in x if i >= 0])


def iou_batch(bb_test, bb_gt):
    """
    From SORT: Computes IOU between two bboxes in the form [x1,y1,x2,y2]
    """
    bb_gt = np.expand_dims(bb_gt, 0)
    bb_test = np.expand_dims(bb_test, 1)

    xx1 = np.maximum(bb_test[..., 0], bb_gt[..., 0])
    yy1 = np.maximum(bb_test[..., 1], bb_gt[..., 1])
    xx2 = np.minimum(bb_test[..., 2], bb_gt[..., 2])
    yy2 = np.minimum(bb_test[..., 3], bb_gt[..., 3])
    w = np.maximum(0., xx2 - xx1)
    h = np.maximum(0., yy2 - yy1)
    wh = w * h
    out = wh / ((bb_test[..., 2] - bb_test[..., 0]) * (bb_test[..., 3] - bb_test[..., 1])
                + (bb_gt[..., 2] - bb_gt[..., 0]) * (bb_gt[..., 3] - bb_gt[..., 1]) - wh)
    return out


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
            [cam_sub, lidar_sub], queue_size=3, slop=0.1)
        sync.registerCallback(self.callback)

        # Create SORT instance for fused detections
        self.tracker = Sort(max_age=4, min_hits=3, iou_threshold=0.01)

        # Create publisher
        self.track_publisher_ = self.create_publisher(
            Detection2DArray, 'image_proc/fusion_tracks', 2)
        self.marker_publisher_ = self.create_publisher(
            MarkerArray, 'lidar_proc/track_markers', 2)

    def callback(self, cam_2d_dets: Detection2DArray, lidar_2d_dets: Detection2DArray):
        """This callback will take the 2 sensors detection arrays and produce a final fused and tracked array

        Args:
            cam_2d_dets (Detection2DArray): Cam detections/tracks
            lidar_2d_dets (Detection2DArray): LiDAR detections/tracks
        """
        t1 = time.clock_gettime(time.CLOCK_THREAD_CPUTIME_ID)

        stamp = cam_2d_dets.header.stamp

        # Convert message arrays to numpy
        cam_dets = detection2DArray2Numpy(cam_2d_dets.detections)
        lidar_dets = detection2DArray2Numpy(lidar_2d_dets.detections)

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
            fused_dets.append(cam_arr[i[0], :])

        unmatched_cam_dets = []
        for i in unmatched_cam_dets_inds:
            unmatched_cam_dets.append(cam_arr[i,:])

        unmatched_lidar_dets = []
        for i in unmatched_lidar_dets_inds:
            unmatched_lidar_dets.append(lidar_arr[i,:])

        return fused_dets + unmatched_cam_dets_inds + unmatched_lidar_dets_inds

    def track2MarkerArray(self, track_ids, stamp) -> MarkerArray:
        m_arr = MarkerArray()
        idx = 0
        for trk in track_ids:
            marker = Marker()
            marker.id = idx
            marker.header.stamp = stamp
            marker.header.frame_id = self.world_frame
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.scale.z = 0.8  # height of `A` in meters
            marker.text = str(int(trk[4]))
            marker.pose.position.x = trk[0]
            marker.pose.position.y = trk[1]
            marker.lifetime = Duration(seconds=0.1).to_msg()
            marker.color.a = 1.0
            marker.color.g = 0.8
            marker.color.b = 0.6

            idx += 1
            m_arr.markers.append(marker)

        return m_arr


def main():
    rclpy.init()
    node = DetectionSyncNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
