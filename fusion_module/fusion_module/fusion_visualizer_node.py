#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge
from cv2 import flip, rectangle, putText, FONT_HERSHEY_PLAIN
from message_filters import ApproximateTimeSynchronizer, Subscriber

class FusionVisualizer(Node):

    def __init__(self):
        super().__init__('fusion_visualizer')

        self.declare_parameter('flip_image', True)
        self.flip_image = self.get_parameter(
            'flip_image').get_parameter_value().bool_value
        
        self.bridge = CvBridge()
        self.image_sub = Subscriber(self, Image, 'image')
        self.detection_sub = Subscriber(self, Detection2DArray, 'image_proc/fusion_tracks')
        self.ats = ApproximateTimeSynchronizer([self.image_sub, self.detection_sub], queue_size=3, slop=0.1)
        self.ats.registerCallback(self.callback)

        self.image_pub = self.create_publisher(Image, 'image_proc/fusion_2D_result', 2)

        self.get_logger().info('Fusion Visualizer started.')

    def callback(self, image_msg, detection_msg):
        # Convert the image message to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')

        if self.flip_image is True:
            cv_image = flip(cv_image, -1)

        # Loop through the detections and draw them on the image
        for detection in detection_msg.detections:
            # Get the bounding box coordinates
            x = int(detection.bbox.center.x - detection.bbox.size_x / 2)
            y = int(detection.bbox.center.y - detection.bbox.size_y / 2)
            w = int(detection.bbox.size_x)
            h = int(detection.bbox.size_y)

            # Draw a rectangle around the detection
            rectangle(cv_image, (x, y), (x + w, y + h), (250, 166, 0), 2)

            # Get the label of the detection
            label = int(detection.results[0].hypothesis.score)

            # Draw the label above the rectangle
            rectangle(cv_image, (x, y + h), (x + 55, y + h + 22), (250, 166, 0), -1)
            putText(cv_image, str(label), (x, y + h + 20), FONT_HERSHEY_PLAIN, 1.3, (255, 255, 255), 2)

        # Convert the image back to ROS format and publish it
        image_with_detections_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        self.image_pub.publish(image_with_detections_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FusionVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
