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
import time

import numpy as np
import rclpy
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sort import Sort
from vision_msgs.msg import ObjectHypothesisWithPose
from vision_msgs.msg import Detection2D, Detection2DArray
from visualization_msgs.msg import Marker, MarkerArray


class DetectionSyncNode(Node):
    def __init__(self):
        super().__init__('detection_sync_node')

        # Get the topic name from the ROS parameter server
        self.world_frame = self.declare_parameter(
            'world_frame', 'map').get_parameter_value().string_value

        cam_track_topic = self.declare_parameter('cam_track_topic', '/image_proc/tracks').get_parameter_value().string_value
        lidar2d_track_topic = self.declare_parameter('lidar2d_track_topic','image_proc/lidar_track_2D').get_parameter_value().string_value

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
        cam_dets = self.detection2DArray2Numpy(cam_2d_dets.detections)
        lidar_dets = self.detection2DArray2Numpy(lidar_2d_dets.detections)

        # Simple fusion with IoU based association
        fused_detections =  cam_dets #self.fuse(cam_dets, lidar_dets)

        # Update SORT with detections
        track_ids = self.tracker.update(fused_detections)

        # Create and Publish 3D Detections with Track IDs
        track_msg_arr = self.createDetection2DArr(
            track_ids, cam_2d_dets.header)
        self.track_publisher_.publish(track_msg_arr)

        # Create and publish Text Marker Array
        m_arr = self.track2MarkerArray(track_ids, stamp)
        self.marker_publisher_.publish(m_arr)

        t2 = time.clock_gettime(time.CLOCK_THREAD_CPUTIME_ID)
        self.get_logger().info('Tracked {:4d} objects in {:.1f} msec.'.format(
            len(m_arr.markers), (t2-t1)*1000))

    def fuse(self, dets1, dets2):
        fused_dets = []

        return fused_dets

    @staticmethod
    def createDetection2DArr(tracks, header) -> Detection2DArray:

        out = Detection2DArray()
        out.header = header

        for trk in tracks:
            det = Detection2D()
            result = ObjectHypothesisWithPose()
            result.hypothesis.score = trk[4]
            det.results.append(result)

            x_len = trk[2] - trk[0]
            y_len = trk[3] - trk[1]

            det.bbox.center.x = x_len/2.0 + trk[0]
            det.bbox.center.y = y_len/2.0 + trk[1]
            det.bbox.size_x = x_len
            det.bbox.size_y = y_len

            out.detections.append(det)

        return out

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
    
    @staticmethod
    def detection2DArray2Numpy(detection_list) -> np.ndarray:
        """Convert vision_msgs/Detection2DArray to numpy array

        Args:
            detection_list (Detection2DArray): Detections

        Returns:
            np.ndarray:
        """
        # v = Detection2D()
        # v.bbox.center.
        if len(detection_list) <= 0:
            return np.zeros((0, 5))

        # Pre-allocate numpy array
        out_arr = np.empty(shape=(len(detection_list), 5), dtype=float)

        for i, det in enumerate(detection_list):
            half_x = det.bbox.size_x/2
            half_y = det.bbox.size_y/2
            out_arr[i] = [
                det.bbox.center.x - half_x,
                det.bbox.center.y - half_y,
                det.bbox.center.x + half_x,
                det.bbox.center.y + half_y,
                0.5]

        return out_arr

def main():
    rclpy.init()
    node = DetectionSyncNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
