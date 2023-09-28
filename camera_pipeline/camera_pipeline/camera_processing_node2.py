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
from camera_processing import vision_track_yolov7
from rclpy.node import Node
from sensor_msgs.msg import Image

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

        # Create Subscriber with callback
        self.subscription = self.create_subscription(
            Image,
            'image',
            self.camera_callback,
            5)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Image subscriber created.')

        # Create Publisher to output annotated results
        self.publisher_ = self.create_publisher(Image, 'image2', 5)

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

        self.tracker = vision_track_yolov7.VisionTracker()
        self.get_logger().info('Vision Tracker created.')

    def camera_callback(self, msg):
        """
        Callback function.
        """
        t1 = time.clock_gettime(time.CLOCK_THREAD_CPUTIME_ID)

        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(msg)

        out_img = cv2.flip(current_frame, -1)

        _, out_img = self.tracker.update(current_frame, return_image=True)

        out_msg = self.br.cv2_to_imgmsg(out_img)
        out_msg.header = msg.header

        self.publisher_.publish(out_msg)

        t5 = time.clock_gettime(time.CLOCK_THREAD_CPUTIME_ID)
        self.get_logger().info(str(t5-t1))


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
