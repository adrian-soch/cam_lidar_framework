"""
 * @file obj_tracker.py
 * @brief Perform 2D Object tracking
 * @author Adrian Sochaniwsky (sochania@mcmaster.ca)
 * @version 0.1
 * @date 2023-03-29

  Note: relative imports only work when running via ROS
    to run via python the relative import must be removed
    Relative import is `.` in front of the imported module
 * 
 * @copyright Copyright (c) 2023
"""

from .obj_tracker_utils import *

import time
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from vision_msgs.msg import Detection3DArray
from visualization_msgs.msg import Marker, MarkerArray

class ObjectTracker(Node):

    def __init__(self):
        super().__init__('object_tracker')

        # Get the topic name from the ROS parameter server
        detection_topic = self.declare_parameter('detection_topic', '/lidar_proc/o_detections').get_parameter_value().string_value
        isOBB = self.declare_parameter('isOBB', True).get_parameter_value().bool_value
        self.world_frame = self.declare_parameter('world_frame', 'map').get_parameter_value().string_value

        if isOBB:
            from .sort import sort_rotated_bbox as s
        else:
            from .sort import sort as s

        #create instance of SORT
        self.tracker = s.Sort(max_age=5, min_hits=3, iou_threshold=0.01)

        self.subscription = self.create_subscription(
            Detection3DArray,
            detection_topic,
            self.callback,
            5)
        self.subscription  # prevent unused variable warning

        self.track_publisher_ = self.create_publisher(Detection3DArray, 'lidar_proc/tracks', 2)
        self.marker_publisher_ = self.create_publisher(MarkerArray, 'lidar_proc/track_markers', 2)      

    def callback(self, msg):
        """Takes new detections and updates the tracker.
        Execution time must be < detection publish rate

        Args:
            msg (vision_msgs/Detection3DArray)
        """
        t1 = time.clock_gettime(time.CLOCK_THREAD_CPUTIME_ID)

        detections = detection3DArray2Numpy(msg.detections)
        
        # update SORT with detections
        track_ids = self.tracker.update(detections)

        # Create and Publish 3D Detections with Track IDs
        track_msg_arr = createDetection3DArr(track_ids, msg.header)
        self.track_publisher_.publish(track_msg_arr)

        # Create and publish Text Marker Array
        m_arr = self.track2MarkerArray(track_ids, msg.header.stamp)
        self.marker_publisher_.publish(m_arr)

        t2 = time.clock_gettime(time.CLOCK_THREAD_CPUTIME_ID)
        self.get_logger().info('Tracked {:4d} objects in {:.1f} msec.'.format(len(m_arr.markers), (t2-t1)*1000))


    def track2MarkerArray(self, track_ids, stamp) -> MarkerArray:
        """
        Create ROS 2 markers for track results

        Returns:
            npArray[x,y,trackID]: _description_
        """

        m_arr = MarkerArray()
        idx = 0
        for trk in track_ids:
            marker = Marker()
            marker.id = idx
            marker.header.stamp = stamp
            marker.header.frame_id = self.world_frame
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.scale.z = 0.8 # height of `A` in meters
            marker.text = str(int(trk[5]))

            marker.pose.position.x = trk[0]
            marker.pose.position.y = trk[1]

            marker.lifetime = Duration(seconds=0.1).to_msg()

            marker.color.a = 1.0
            marker.color.g = 0.8
            marker.color.b = 0.6

            idx += 1
            m_arr.markers.append(marker)

        return m_arr

def main(args=None):
    rclpy.init(args=args)
    object_tracker = ObjectTracker()
    rclpy.spin(object_tracker)

    object_tracker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
