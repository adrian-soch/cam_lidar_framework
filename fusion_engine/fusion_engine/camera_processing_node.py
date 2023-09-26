#! /usr/bin/env python3
"""
@file camera_processing_node.py

@brief This node subsribes to images and runs detection+tracking

@section TODO
- Add debug flag to remove log statemetns when in "production mode"
- Add flag to remove confidence from output

@section Author(s)
- Created by Adrian Sochaniwsky on 13/11/2022
"""
import cv2
import time
import rclpy
from camera_processing import vision_track
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import ObjectHypothesisWithPose
from vision_msgs.msg import Detection2D, Detection2DArray

from cv_bridge import CvBridge


class ImageSubscriber(Node):
    """
    Create an ImageSubscriber class, which is a subclass of the Node class.
    """

    def __init__(self):
        """
        Class constructor to set up the node
        """
        super().__init__('camera_processor')
        self.declare_parameter('flip_image', False)
        self.flip_image = self.get_parameter(
            'flip_image').get_parameter_value().bool_value

        # Create Subscriber with callback
        self.subscription = self.create_subscription(
            Image,
            'image',
            self.camera_callback,
            5)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Image subscriber created.')

        # Create Publisher to output annotated results
        self.result_pub_ = self.create_publisher(Image, 'image_proc/result', 5)
        self.track_pub_ = self.create_publisher(
            Detection2DArray, 'image_proc/tracks', 5)

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

        self.tracker = vision_track.VisionTracker()
        self.get_logger().info('Vision Tracker created.')

    def camera_callback(self, msg):
        """
        Callback function.
        """
        t1 = time.clock_gettime(time.CLOCK_THREAD_CPUTIME_ID)

        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(msg)

        if self.flip_image is True:
            current_frame = cv2.flip(current_frame, -1)

        tracks, out_img = self.tracker.update(current_frame, return_image=True)

        # Pub track results
        track_msg_arr = self.createDetection2DArr(tracks, msg.header)
        self.track_pub_.publish(track_msg_arr)

        # Pub visual results
        out_msg = self.br.cv2_to_imgmsg(out_img)
        out_msg.header = msg.header
        self.result_pub_.publish(out_msg)

        t5 = time.clock_gettime(time.CLOCK_THREAD_CPUTIME_ID)
        self.get_logger().info(f'Time (msec): {(t5-t1)*1000:.1f}')

    @staticmethod
    def createDetection2DArr(tracks, header) -> Detection2DArray:
        """Convert tracker output to message for publishing

        Args:
            tracks (ndarray): Array of the form [[x1,y1,x2,y2,id], [x1,y1,x2,y2,id], ...]

        Returns:
            Detection2DArray
        """
        out = Detection2DArray()
        out.header = header

        for trk in tracks:
            if trk is None:
                continue

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


if __name__ == '__main__':
    # Initialize the rclpy library
    rclpy.init(args=None)

    # Create the node
    camera_processor = ImageSubscriber()

    # Spin the node so the callback function is called.
    rclpy.spin(camera_processor)
    camera_processor.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()
