#!/usr/bin/env python3
"""
This node subscribes to a Detection2DArray which stores tracker results.
It will convert the Detection2D object into a csv entry and save the csv file

This was created to save the results for offline evaluation.
"""
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray

from rclpy.time import Time

FOLDER_PATH = '/home/adrian/dev/metrics/SORT_Results'


class MotEntry:
    """Contains the values and helper functions for the detections in MOT Challenge format
        See comment at the top of the file for more details.
    """

    def __init__(self, frame, id=None, bb_left=None, bb_top=None, bb_width=None, bb_height=None, conf=-1, x=-1, y=-1, z=-1):
        self.frame = frame
        self.id = id
        self.bb_left = bb_left
        self.bb_top = bb_top
        self.bb_width = bb_width
        self.bb_height = bb_height
        self.conf = conf
        self.x = x
        self.y = y
        self.z = z

    def toStr(self):
        return "{},{},{:.1f},{:.1f},{:.1f},{:.1f},{},{},{},{}".format(
            self.frame, self.id, self.bb_left, self.bb_top, self.bb_width, self.bb_height,
            self.conf, self.x, self.y, self.z)


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

        topic = 'image_proc/fusion_tracks'  # fusion_topic

        # Get the topic name from the ROS parameter server
        topic_name = self.declare_parameter(
            'topic_name', topic).get_parameter_value().string_value

        # Subscribe to the detection messages
        self.subscription = self.create_subscription(
            Detection2DArray, topic_name, self.detection_callback, 2)

        # Get the current date and time
        now = datetime.now()
        time = now.strftime("%Y-%m-%d_%H-%M-%S")
        self.out_path = os.path.join(FOLDER_PATH, time + '_fusion_output.txt')
        create_folder_path(self.out_path)

        self.frame_count = 0

        self.get_logger().info('Fusion detection 2 CSV node initialized.')

    def detection_callback(self, msg):
        self.get_logger().debug('Got detections.')
        self.frame_count += 1

        # Hold all the tracks in a frame
        tracks = []

        if len(msg.detections) > 0:
            for detection in msg.detections:
                # Get the position and size of the detection
                pos = detection.bbox.center
                size = detection.bbox

                # ts = detection.header.stamp
                time = Time.from_msg(msg.header.stamp)
                sec, nsec = time.seconds_nanoseconds()

                result = detection.results[0]
                id = int(result.hypothesis.score)

                entry = MotEntry(self.frame_count, id,
                                 bb_left=pos.x - size.size_x/2.0,
                                 bb_top=pos.y - size.size_y/2.0,
                                 bb_width=size.size_x,
                                 bb_height=size.size_y,
                                 z=sec + nsec/1e9)
                tracks.append(entry)
        else:
            # Add placeholder for frames without detection
            entry = MotEntry(self.frame_count, -1,
                             bb_left=-1,
                             bb_top=-1,
                             bb_width=-1,
                             bb_height=-1)
            tracks.append(entry)

        # Print to txt file
        with open(self.out_path, "a") as f:
            for trk in tracks:
                f.write(trk.toStr() + "\n")


def main(args=None):
    rclpy.init(args=args)
    detector_node = DetectorNode()
    rclpy.spin(detector_node)
    detector_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
