#! /usr/bin/env python3
"""
@file fusion_3d_node.py

@brief This node subscribes to 3D detections/tracks from camera and lidar sensors
    It will associate dets based on euclidean distance between 3D centroids.
    Unmatched dets + matched dets list will be tracked via modified SORT MOT tracker.
    Final results will be published as a Detection3DArray.

@section Author(s)
- Created by Adrian Sochaniwsky on 25/09/2023
"""

import os
os.environ["OMP_NUM_THREADS"] = "1"
os.environ["OPENBLAS_NUM_THREADS"] = "1"
os.environ["MKL_NUM_THREADS"] = "1"
os.environ["VECLIB_MAXIMUM_THREADS"] = "1"
os.environ["NUMEXPR_NUM_THREADS"] = "1"

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from message_filters import ApproximateTimeSynchronizer, Subscriber
from vision_msgs.msg import Detection3DArray
from visualization_msgs.msg import Marker, MarkerArray

import numpy as np
import random
import time

from fusion_3d.utils import createDetection3DArr, detection3DArray2Numpy
from fusion_3d.sort_modified import Sort, linear_assignment, iou_rotated_bbox, state2polygon


class DetectionSyncNode(Node):
    def __init__(self):
        super().__init__('detection_sync_node')

        self.world_frame = self.declare_parameter(
            'world_frame', 'map').get_parameter_value().string_value

        cam_track_topic = self.declare_parameter(
            'cam_track_topic', 'image_proc/det3D_tracks').get_parameter_value().string_value
        lidar3d_track_topic = self.declare_parameter(
            'lidar3d_track_topic', 'lidar_proc/tracks').get_parameter_value().string_value

        tracker_iou_thresh = self.declare_parameter(
            'tracker_iou_thresh', 0.001).get_parameter_value().double_value

        self.fusion_iou_thresh = self.declare_parameter(
            'fusion_iou_thresh', 0.001).get_parameter_value().double_value

        self.lidar_only_override = self.declare_parameter(
            'lidar_only_override', False).get_parameter_value().bool_value

        self.camera_only_override = self.declare_parameter(
            'camera_only_override', False).get_parameter_value().bool_value

        if self.lidar_only_override and self.camera_only_override:
            self.get_logger().error('Sensor overrrides are mutally exclusive, check parameters.')
            exit(-1)

        # Create subscribers and the approximate syncronizer message filter
        cam_sub = Subscriber(self, Detection3DArray, cam_track_topic)
        lidar_sub = Subscriber(self, Detection3DArray, lidar3d_track_topic)
        sync = ApproximateTimeSynchronizer(
            [cam_sub, lidar_sub], queue_size=10, slop=0.1)
        sync.registerCallback(self.callback)

        # Create SORT instance for fused detections
        self.tracker = Sort(max_age=2, min_hits=3,
                            threshold=tracker_iou_thresh)

        # Create publishers
        self.track_publisher_ = self.create_publisher(
            Detection3DArray, 'image_proc/fusion_3d_tracks', 2)

        # Visualization markers
        self.bbox_publisher = self.create_publisher(
            MarkerArray, 'fusion/bbox_3d', 2)
        self.tracklet_publisher = self.create_publisher(
            MarkerArray, 'fusion/tracklet_3d', 2)
        self.marker_publisher = self.create_publisher(
            MarkerArray, 'fusion/track_ID_3d', 2)

        # Index to prevent us from overwriting a tracklet that we
        # still want to see in Rviz
        self.tracklet_idx = 0

        self.get_logger().info('Fusion Module initialized.')

    def callback(self, cam_3d_dets: Detection3DArray, lidar_3d_dets: Detection3DArray):
        """This callback will take the 2 sensors detection arrays and produce a final fused and tracked array

        Args:
            cam_3d_dets (Detection3DArray): Cam detections/tracks
            lidar_3d_dets (Detection3DArray): LiDAR detections/tracks
        """
        t1 = time.clock_gettime(time.CLOCK_THREAD_CPUTIME_ID)

        # Convert message arrays to numpy
        cam_dets = detection3DArray2Numpy(
            cam_3d_dets.detections, sensor_type='C')
        lidar_dets = detection3DArray2Numpy(
            lidar_3d_dets.detections, sensor_type='L')

        if self.camera_only_override:
            fused_detections = cam_dets
        elif self.lidar_only_override:
            fused_detections = lidar_dets
        else:
            # Simple fusion with IoU based association
            fused_detections = self.fuse(
                cam_arr=cam_dets, lidar_arr=lidar_dets, dist_threshold=self.fusion_iou_thresh)

        # Update SORT with detections
        track_ids = self.tracker.update(fused_detections)

        # Create and Publish 3D Detections with Track IDs
        track_msg_arr = createDetection3DArr(track_ids, cam_3d_dets.header)
        self.track_publisher_.publish(track_msg_arr)

        id_text_array, track_bbox_array, tracklet_array = self.track2MarkerArrays(
            track_msg_arr.detections)
        self.marker_publisher.publish(id_text_array)
        self.bbox_publisher.publish(track_bbox_array)
        self.tracklet_publisher.publish(tracklet_array)

        # Get and publish the execution time
        t2 = time.clock_gettime(time.CLOCK_THREAD_CPUTIME_ID)
        self.get_logger().info('Tracked {:4d} objects in {:.1f} msec.'.format(
            len(track_msg_arr.detections), (t2-t1)*1000))

    def fuse(self, cam_arr, lidar_arr, dist_threshold=3.0) -> np.ndarray:
        """Perform late fusion between 2 sets of Axis-aligned bounding boxes from 2 different sensors

        Associate the 2 lists. For all matched detections just keep the camera detection.
        Keep all unmatched detections. This will use all camera detections
        including the longer distance and also use lidar detections in
        poor conditions the camera didnt get

        NOTE: Code is adapted from SORT algorithm

        Args:
            cam_arr (np.ndarray): Cam detection array
            lidar_arr (np.ndarray): Lidar detection array
            dist_threshold (float, optional): Threshold for euclidean-based
                association. Defaults to 3.0 m.

        Returns:
            np.ndarray: list of fused filt_match_inds and unmatched detection from both sensors
        """

        # Get NxN matrix of IoU between detections
        iou_matrix = np.zeros((len(cam_arr), len(lidar_arr)), dtype=np.float32)
        det_poly_array = []
        for det in cam_arr:
            det_poly_array.append(state2polygon(det))

        trk_poly_array = []
        for trk in lidar_arr:
            trk_poly_array.append(state2polygon(trk))

        for d in range(len(cam_arr)):
            for t in range(len(lidar_arr)):
                iou_matrix[d, t] = iou_rotated_bbox(
                    det_poly_array[d], trk_poly_array[t])

        # Find all filt_match_inds
        if min(iou_matrix.shape) > 0:
            a = (iou_matrix > dist_threshold).astype(np.int32)
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
            if (iou_matrix[m[0], m[1]] < dist_threshold):
                unmatched_cam_dets_inds.append(m[0])
                unmatched_lidar_dets_inds.append(m[1])
            else:
                filt_match_inds.append(m.reshape(1, 2))
        if (len(filt_match_inds) == 0):
            filt_match_inds = np.empty((0, 2), dtype=int)
        else:
            filt_match_inds = np.concatenate(filt_match_inds, axis=0)

        fused_dets = np.empty((0, 8))
        for i in filt_match_inds:
            # Change sensor type to 'B' (both)
            det = lidar_arr[i[1], :]
            det[-1] = float(ord('B'))
            fused_dets = np.vstack([fused_dets, lidar_arr[i[1], :]])

        unmatched_cam_dets = np.empty((0, 8))
        for i in unmatched_cam_dets_inds:
            unmatched_cam_dets = np.vstack([unmatched_cam_dets, cam_arr[i, :]])

        unmatched_lidar_dets = np.empty((0, 8))
        for i in unmatched_lidar_dets_inds:
            unmatched_lidar_dets = np.vstack([unmatched_lidar_dets, lidar_arr[i, :]])

        return np.vstack([fused_dets,unmatched_cam_dets,unmatched_lidar_dets])

    def track2MarkerArrays(self, dets: Detection3DArray) -> [MarkerArray, MarkerArray, MarkerArray]:
        """Create ROS 2 markers for track results

        Args:
            dets: Detection3DArray

        Returns:
            id_text_array,      Marker arrays
            track_bbox_array,
            tracklet_array
        """
        id_text_array = MarkerArray()
        track_bbox_array = MarkerArray()
        tracklet_array = MarkerArray()
        for idx, det in enumerate(dets):

            # Get track ID
            trk_id = det.results[0].hypothesis.score

            # Get a colour based on the track ID
            # set the seed value so the same colour is applied
            # to the same track each time
            random.seed(trk_id)
            r, g, b = random.random(), random.random(), random.random()

            # ID Text Markers
            marker = Marker()
            marker.id = idx
            marker.header = det.header
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.scale.z = 0.8  # height of 'A' in meters
            marker.text = str(int(trk_id))
            marker.pose.position = det.bbox.center.position
            marker.lifetime = Duration(seconds=0.1).to_msg()
            marker.color.a = 1.0
            marker.color.g = 0.8
            marker.color.b = 0.6
            id_text_array.markers.append(marker)

            # 3D BBox Markers
            marker = Marker()
            marker.id = idx
            marker.header = det.header
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position = det.bbox.center.position
            marker.scale = det.bbox.size
            marker.pose.orientation = det.bbox.center.orientation
            marker.color.a = 0.6
            marker.color.r = r
            marker.color.g = g
            marker.color.b = b
            marker.lifetime = Duration(seconds=0.1).to_msg()
            track_bbox_array.markers.append(marker)

            # Tracklet Markers
            tracklet = Marker()
            tracklet.id = self.tracklet_idx
            tracklet.header = det.header
            tracklet.type = Marker.SPHERE
            tracklet.action = Marker.ADD
            tracklet.pose.position = det.bbox.center.position
            tracklet.scale.x = 0.3
            tracklet.scale.y = 0.3
            tracklet.scale.z = 0.3
            tracklet.color.a = 0.6
            tracklet.color.r = r
            tracklet.color.g = g
            tracklet.color.b = b
            tracklet.lifetime = Duration(seconds=1.5).to_msg()
            self.tracklet_idx = (self.tracklet_idx + 1) % 400000
            tracklet_array.markers.append(tracklet)

        return id_text_array, track_bbox_array, tracklet_array


def main():
    rclpy.init()
    node = DetectionSyncNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
