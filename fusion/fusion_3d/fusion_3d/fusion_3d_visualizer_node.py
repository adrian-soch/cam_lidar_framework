#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge
from cv2 import flip, rectangle, putText, FONT_HERSHEY_PLAIN
from cv2 import VideoWriter, VideoWriter_fourcc
from message_filters import ApproximateTimeSynchronizer, Subscriber

import datetime


class FusionVisualizer(Node):

    def __init__(self):
        super().__init__('fusion_visualizer')

        self.declare_parameter('flip_image', True)
        self.flip_image = self.get_parameter(
            'flip_image').get_parameter_value().bool_value

        self.declare_parameter('save_video', False)
        save_video = self.get_parameter(
            'save_video').get_parameter_value().bool_value

        self.declare_parameter('video_name', 'video')
        self.video_name = self.get_parameter(
            'video_name').get_parameter_value().string_value

        self.bridge = CvBridge()
        self.image_sub = Subscriber(self, Image, 'image')
        self.detection_sub = Subscriber(
            self, Detection2DArray, 'image_proc/fusion_tracks')
        self.ats = ApproximateTimeSynchronizer(
            [self.image_sub, self.detection_sub], queue_size=3, slop=0.1)
        self.ats.registerCallback(self.callback)

        self.image_pub = self.create_publisher(
            Image, 'image_proc/fusion_2D_result', 2)

        # BGR Colours
        self.colour_codes = [(20, 230,  0),  # C - camera
                             (0, 140, 255),  # L - lidar
                             (255, 140, 0),  # B - both
                             (0,  0, 202)]   # U - unknown

        if save_video is True:
            self.video_writer = None
        else:
            self.video_writer = False
        self.video_writer_activated = False
        self.get_logger().info('Fusion Visualizer started.')

    def callback(self, image_msg, detection_msg):
        # Convert the image message to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(
            image_msg, desired_encoding='bgr8')

        if self.flip_image is True:
            cv_image = flip(cv_image, -1)

        # Loop through the detections and draw them on the image
        for detection in detection_msg.detections:
            # Get the bounding box coordinates
            try:
                x = int(detection.bbox.center.position.x - detection.bbox.size_x / 2)
                y = int(detection.bbox.center.position.y - detection.bbox.size_y / 2)
                w = int(detection.bbox.size_x)
                h = int(detection.bbox.size_y)
            except ValueError:
                continue

            # Indicates which sensor contributed to this track result
            colour = None
            sensor_indicator = detection.results[0].hypothesis.class_id

            if sensor_indicator == 'C':
                # For camera only
                colour = self.colour_codes[0]
            elif sensor_indicator == 'L':
                # For LiDAR only
                colour = self.colour_codes[1]
            elif sensor_indicator == 'B':
                # For both sensors
                colour = self.colour_codes[2]
            else:
                colour = self.colour_codes[3]

            # Draw a rectangle around the detection
            rectangle(cv_image, (x, y), (x + w, y + h), (40, 40, 250), 2)

            # Get the track_id of the detection
            track_id = int(detection.results[0].hypothesis.score)

            # Draw the label above the rectangle
            rectangle(cv_image, (x, y + h), (x + 55, y + h + 22), colour, -1)
            putText(cv_image, f'{sensor_indicator}-{track_id}', (x, y +
                    h + 20), FONT_HERSHEY_PLAIN, 1.3, (255, 255, 255), 2)

        # Convert the image back to ROS format and publish it
        image_with_detections_msg = self.bridge.cv2_to_imgmsg(
            cv_image, encoding='bgr8')
        self.image_pub.publish(image_with_detections_msg)

        # create video writer if not exists
        if self.video_writer is None:
            fourcc = VideoWriter_fourcc(*'mp4v')  # choose codec
            fps = 10  # choose frame rate
            filename = self.video_name + '_{}.mp4'.format(datetime.datetime.now().strftime(
                '%Y%m%d_%H%M%S'))  # choose file name with current timestamp
            self.video_writer = VideoWriter(
                filename, fourcc, fps, (cv_image.shape[1], cv_image.shape[0]))
            self.video_writer_activated = True
        elif self.video_writer_activated is True:
            # write image to video file
            self.video_writer.write(cv_image)


def main(args=None):
    rclpy.init(args=args)
    node = FusionVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
