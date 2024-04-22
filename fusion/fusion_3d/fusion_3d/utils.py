
import numpy as np
from tf_transformations import quaternion_from_euler
from vision_msgs.msg import Detection3D, Detection3DArray
from vision_msgs.msg import ObjectHypothesisWithPose


def detection3DArray2Numpy(detection_list, sensor_type) -> np.ndarray:
    """Convert vision_msgs/Detection3DArray to numpy array

    Args:
        detection_list (Detection3DArray): Detections

    Returns:
        np.ndarray: [x,y,z,yaw,l,h,w]
    """
    entry_size = 8
    if len(detection_list) <= 0:
        return np.zeros((0, entry_size))

    # Pre-allocate numpy array
    out_arr = np.empty(shape=(len(detection_list), entry_size), dtype=float)

    for i, det in enumerate(detection_list):
        out_arr[i] = [
            det.bbox.center.position.x,
            det.bbox.center.position.y,
            det.bbox.center.position.z,
            euler_from_quaternion(det.bbox.center.orientation),
            det.bbox.size.x,
            det.bbox.size.y,
            det.bbox.size.y,
            float(ord(sensor_type))
        ]

    return out_arr


def createDetection3DArr(tracks, header) -> Detection3DArray:
    """Convert tracker results to Detection3DArray msg

    Args:
        tracks (np.ndarray): Tracker output: [x,y,z,yaw,l,h,w,vx,vy,dw,ID,sensorType]
        header (ROS msg header): Stamp and frame

    Returns:
        Detection2DArray: output message
    """

    out = Detection3DArray()
    out.header = header

    for trk in tracks:
        det = Detection3D()
        det.header = header
        result = ObjectHypothesisWithPose()
        result.hypothesis.score = trk[-2]
        result.hypothesis.class_id = chr(int(trk[-1]))
        det.results.append(result)

        det.bbox.center.position.x = trk[0]
        det.bbox.center.position.y = trk[1]
        det.bbox.center.position.z = trk[2]

        q = quaternion_from_euler(0, 0, trk[3])

        det.bbox.center.orientation.x = q[0]
        det.bbox.center.orientation.y = q[1]
        det.bbox.center.orientation.z = q[2]
        det.bbox.center.orientation.w = q[3]
        det.bbox.size.x = trk[4]
        det.bbox.size.y = trk[5]
        det.bbox.size.z = trk[6]

        out.detections.append(det)

    return out


def euler_from_quaternion(q):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)

    Note: only returns yaw about z axis
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

    return yaw_z  # in radians
