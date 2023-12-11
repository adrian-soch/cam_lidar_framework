
import numpy as np

from vision_msgs.msg import Detection2D, Detection2DArray
from vision_msgs.msg import Detection3D, Detection3DArray
from vision_msgs.msg import ObjectHypothesisWithPose


def detection2DArray2Numpy(detection_list, sensor_type) -> np.ndarray:
    """Convert vision_msgs/Detection2DArray to numpy array

    Args:
        detection_list (Detection2DArray): Detections

    Returns:
        np.ndarray: [x_min, y_min, x_max, y_max, confidence]

        Note: 'confidence' is not used.
    """
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
            float(ord(sensor_type))]

    return out_arr


def createDetection2DArr(tracks, header) -> Detection2DArray:
    """Convert tracker results to Detection2DArray msg

    Args:
        tracks (np.ndarray): Tracker output
        header (ROS msg header): Stamp and frame

    Returns:
        Detection2DArray: output message
    """

    out = Detection2DArray()
    out.header = header

    for trk in tracks:
        det = Detection2D()
        result = ObjectHypothesisWithPose()
        result.hypothesis.score = trk[4]
        result.hypothesis.class_id = chr(int(trk[5]))
        det.results.append(result)

        x_len = trk[2] - trk[0]
        y_len = trk[3] - trk[1]

        det.bbox.center.x = x_len/2.0 + trk[0]
        det.bbox.center.y = y_len/2.0 + trk[1]
        det.bbox.size_x = x_len
        det.bbox.size_y = y_len

        out.detections.append(det)

    return out


def detection3DArray2Numpy(detection_list, sensor_type) -> np.ndarray:
    """Convert vision_msgs/Detection3DArray to numpy array

    Args:
        detection_list (Detection3DArray): Detections

    Returns:
        np.ndarray: [x_min, y_min, x_max, y_max, confidence]

        Note: 'confidence' is not used.
    """
    if len(detection_list) <= 0:
        return np.zeros((0, 5))

    # Pre-allocate numpy array
    out_arr = np.empty(shape=(len(detection_list), 5), dtype=float)

    # for i, det in enumerate(detection_list):
    #     half_x = det.bbox.size_x/2
    #     half_y = det.bbox.size_y/2
    #     out_arr[i] = [
    #         det.bbox.center.x - half_x,
    #         det.bbox.center.y - half_y,
    #         det.bbox.center.x + half_x,
    #         det.bbox.center.y + half_y,
    #         float(ord(sensor_type))]

    return out_arr


def createDetection3DArr(tracks, header) -> Detection3DArray:
    """Convert tracker results to Detection3DArray msg

    Args:
        tracks (np.ndarray): Tracker output
        header (ROS msg header): Stamp and frame

    Returns:
        Detection2DArray: output message
    """

    out = Detection3DArray()
    out.header = header

    # for trk in tracks:
    #     det = Detection3D()
    #     result = ObjectHypothesisWithPose()
    #     result.hypothesis.score = trk[4]
    #     result.hypothesis.class_id = chr(int(trk[5]))
    #     det.results.append(result)

    #     x_len = trk[2] - trk[0]
    #     y_len = trk[3] - trk[1]

    #     det.bbox.center.x = x_len/2.0 + trk[0]
    #     det.bbox.center.y = y_len/2.0 + trk[1]
    #     det.bbox.size_x = x_len
    #     det.bbox.size_y = y_len

    #     out.detections.append(det)

    return out
