import numpy as np

from vision_msgs.msg import ObjectHypothesisWithPose
from vision_msgs.msg import Detection3D, Detection3DArray

from tf_transformations import quaternion_from_euler

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