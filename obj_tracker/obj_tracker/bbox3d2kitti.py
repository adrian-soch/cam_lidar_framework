"""
This node subscribes to a Detection3DArray into Kitti 3D detection results

1 file per frame, each detection is in the following format:

[class, x, y, z, l, w, h, yaw]

This was created to save the results for offline evaluation.
"""
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection3DArray

import numpy as np

FOLDER_PATH = '/home/adrian/dev/metrics/Kitti_Results'


def create_folder_path(file_path):
    """
    Creates the folders in the given file path if they don't already exist.
    """
    folder_path = os.path.dirname(file_path)
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)


class DetectorNode(Node):
    def __init__(self):
        super().__init__('detector_node')

        # Get the topic name from the ROS parameter server
        topic_name = self.declare_parameter(
            'topic_name', '/lidar_proc/tracks').get_parameter_value().string_value

        # Subscribe to the detection messages
        self.subscription = self.create_subscription(
            Detection3DArray, topic_name, self.detection_callback, 5)

        # Get the current date and time
        now = datetime.now()
        self.time = now.strftime("%Y-%m-%d_%H-%M-%S")
        self.frame_count = 0

        self.get_logger().info(f'3D Track logger initialized.')

    def detection_callback(self, msg):
        self.get_logger().info(f'Saving frame {str(self.frame_count)} results.')

        # Hold all the tracks in a frame
        tracks = []

        # Loop through each detection
        for detection in msg.detections:
            # Get the position and size of the detection
            pos = detection.bbox.center.position
            orientation = detection.bbox.center.orientation
            size = detection.bbox.size
            obj_class = int(float(detection.id))
            yaw = euler_from_quaternion(
                orientation.w, orientation.x, orientation.y, orientation.z)

            entry = f'{obj_class:05d} {pos.x:.3f} {pos.y:.3f} {pos.z:.3f} {size.x:.3f} {size.y:.3f} {size.z:.3f} {yaw:.3f}'
            tracks.append(entry)

        # Print to txt file
        out_path = os.path.join(
            FOLDER_PATH,self.time, str(self.frame_count) + '_' +  'kitti.txt')
        if self.frame_count <= 0:
            create_folder_path(out_path)
        with open(out_path, "w") as f:
            for trk in tracks:
                f.write(trk + "\n")
        self.frame_count += 1


def euler_from_quaternion(qw, qx, qy, qz):
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

    t3 = +2.0 * (qw * qz + qx * qy)
    t4 = +1.0 - 2.0 * (qy * qy + qz * qz)
    yaw_z = np.arctan2(t3, t4)

    return yaw_z  # in radians


def main(args=None):
    rclpy.init(args=args)
    detector_node = DetectorNode()
    rclpy.spin(detector_node)
    detector_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
