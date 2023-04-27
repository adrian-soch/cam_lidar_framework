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

from .sort import sort

import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from vision_msgs.msg import Detection3DArray
from visualization_msgs.msg import MarkerArray, Marker

WORLD_FRAME = 'map'

class ObjectTracker(Node):

    def __init__(self):
        super().__init__('object_tracker')
        self.subscription = self.create_subscription(
            Detection3DArray,
            'detections',
            self.callback,
            5)
        self.subscription  # prevent unused variable warning

        self.publisher_ = self.create_publisher(MarkerArray, 'track_markers', 2)

        #create instance of SORT
        self.tracker = sort.Sort()
        

    def callback(self, msg):
        """Takes new detections and updates the tracker.
        Execution time must be < detection publish rate

        Args:
            msg (vision_msgs/Detection3DArray)
        """
        t1 = time.clock_gettime(time.CLOCK_THREAD_CPUTIME_ID)

        detections = detection3DArray2Numpy(msg.detections)
        
        # update SORT with detections
        # track_bbs_ids is a np array where each row contains a valid bounding box and track_id (last column)
        track_ids = self.tracker.update(detections)

        # Create and publish Text Marker Array
        m_arr = track2MarkerArray(track_ids, msg.header.stamp)
        self.publisher_.publish(m_arr)

        # with np.printoptions(precision=3, suppress=True):
        #     print(track_ids)

        t2 = time.clock_gettime(time.CLOCK_THREAD_CPUTIME_ID)
        self.get_logger().info('Tracked {:4d} objects in {:.1f} msec.'.format(len(m_arr.markers), (t2-t1)*1000))


def detection3DArray2Numpy(detection_list):
    """
    Convert vision_msgs/Detection3DArray to numpy array

    Args:
        detection_list (vision_msgs/Detection3DArray)

    Returns:
        numpy_arr: Numpy float array of [x1,y1,x2,y2,score].
    """
    # Pre-allocate numpy array
    out_arr = np.empty(shape=(len(detection_list),5), dtype=float)

    idx = 0
    for det in detection_list:
        half_x = det.bbox.size.x/2
        half_y = det.bbox.size.y/2
        out_arr[idx] = [
            det.bbox.center.position.x - half_x,
            det.bbox.center.position.y - half_y,
            det.bbox.center.position.x + half_x,
            det.bbox.center.position.y + half_y,
            0.5]
        idx += 1

    return out_arr

def detection3DArray2Numpy2(detection_list):
    """
    Convert vision_msgs/Detection3DArray to numpy array

    Args:
        detection_list (vision_msgs/Detection3DArray)

    Returns:
        numpy_arr: Numpy float array of [x,y,z,h/l,h/w]. Position of bounding box x,y,z 
            and ratio of height/length, height/width of the bounding box
    """
    # Pre-allocate numpy array
    out_arr = np.empty(shape=(len(detection_list),5), dtype=float)

    idx = 0
    for det in detection_list:
        out_arr[idx] = [det.bbox.center.position.x,
            det.bbox.center.position.y,
            det.bbox.center.position.z,
            det.bbox.size.z/det.bbox.size.x,
            det.bbox.size.z/det.bbox.size.y]
        idx += 1

    return out_arr

def quat2yaw(w,z) -> float:
    """Convert quaternion to yaw angle
        Assumes quat is normalized, and x,y =0 0

    Args:
        w (float): w from unit quaternion
        z (float): z from unit quaternion

    Returns:
        float: yaw angle in radians
    """
    return np.arctan2(2.0 * w * z)

def track2MarkerArray(track_ids, stamp) -> MarkerArray:
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
        marker.header.frame_id = WORLD_FRAME
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.scale.z = 0.8 # height of `A` in meters
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

def main(args=None):
    rclpy.init(args=args)
    object_tracker = ObjectTracker()
    rclpy.spin(object_tracker)

    object_tracker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
