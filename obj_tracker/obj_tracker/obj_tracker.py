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

from sort import sort_rotated_bbox

import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from vision_msgs.msg import ObjectHypothesisWithPose
from visualization_msgs.msg import Marker, MarkerArray
from vision_msgs.msg import Detection3D, Detection3DArray

from tf_transformations import quaternion_from_euler

WORLD_FRAME = 'map'

class ObjectTracker(Node):

    def __init__(self):
        super().__init__('object_tracker')
        self.subscription = self.create_subscription(
            Detection3DArray,
            'lidar_proc/detections',
            self.callback,
            5)
        self.subscription  # prevent unused variable warning

        self.track_publisher_ = self.create_publisher(Detection3DArray, 'lidar_proc/tracks', 2)
        self.marker_publisher_ = self.create_publisher(MarkerArray, 'lidar_proc/track_markers', 2)

        #create instance of SORT
        self.tracker = sort_rotated_bbox.Sort(max_age=5, min_hits=3, iou_threshold=0.01)
        

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
        m_arr = track2MarkerArray(track_ids, msg.header.stamp)
        self.marker_publisher_.publish(m_arr)

        t2 = time.clock_gettime(time.CLOCK_THREAD_CPUTIME_ID)
        self.get_logger().info('Tracked {:4d} objects in {:.1f} msec.'.format(len(m_arr.markers), (t2-t1)*1000))

def createDetection3DArr(tracks, header) -> Detection3DArray:
    """Convert tracker output to message for publishing

    Args:
        tracks (ndarray): Array of the form [[x,y,x*y,w/h, angle], [x,y,x*y,w/h, angle], ...]

    Returns:
        Detection3DArray:
    """
    out = Detection3DArray()
    out.header = header

    for trk in tracks:
        det = Detection3D()
        result = ObjectHypothesisWithPose()
        result.hypothesis.score = trk[5]
        det.results.append(result)

        y_len = np.sqrt(trk[2]*trk[3])
        x_len = trk[2]/y_len

        det.bbox.center.position.x = trk[0]
        det.bbox.center.position.y = trk[1]
        det.bbox.size.x = x_len
        det.bbox.size.y = y_len
        
        q = quaternion_from_euler(0, 0, trk[4])
        det.bbox.center.orientation.x = q[0]
        det.bbox.center.orientation.y = q[1]
        det.bbox.center.orientation.z = q[2]
        det.bbox.center.orientation.w = q[3]

        out.detections.append(det)
    return out


def detection3DArray2Numpy(detection_list):
    """
    Convert vision_msgs/Detection3DArray to numpy array

    Args:
        detection_list (vision_msgs/Detection3DArray)

    Returns:
        numpy_arr: Numpy float array of [[x,y,x*y,w/h, angle], [...], ...]
    """
    if len(detection_list) <= 0:
        return np.empty((0, 5))

    # Pre-allocate numpy array
    out_arr = np.empty(shape=(len(detection_list),5), dtype=float)

    idx = 0
    for det in detection_list:
        area = det.bbox.size.y*det.bbox.size.x
        angle = euler_from_quaternion(det.bbox.center.orientation)
        out_arr[idx] = [det.bbox.center.position.x, det.bbox.center.position.y,
                                 area,
                                 det.bbox.size.y/det.bbox.size.x,
                                 angle]
        idx += 1

    return out_arr

def euler_from_quaternion(q):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        # t0 = +2.0 * (q.w * q.x + q.y * q.z)
        # t1 = +1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        # roll_x = np.atan2(t0, t1)
     
        # t2 = +2.0 * (q.w * q.y - q.z * q.x)
        # t2 = +1.0 if t2 > +1.0 else t2
        # t2 = -1.0 if t2 < -1.0 else t2
        # pitch_y = np.asin(t2)
     
        t3 = +2.0 * (q.w * q.z + q.x * q.y)
        t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw_z = np.arctan2(t3, t4)
     
        return yaw_z # in radians

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
