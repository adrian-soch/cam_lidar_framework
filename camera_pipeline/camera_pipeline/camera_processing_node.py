#! /usr/bin/env python3
"""
@file camera_processing_node.py

@brief This node subsribes to images and runs object detection

@section Author(s)
- Created by Adrian Sochaniwsky on 13/11/2022
"""

# limit the number of cpus used by high performance libraries
import os
os.environ["OMP_NUM_THREADS"] = "1"
os.environ["OPENBLAS_NUM_THREADS"] = "1"
os.environ["MKL_NUM_THREADS"] = "1"
os.environ["VECLIB_MAXIMUM_THREADS"] = "1"
os.environ["NUMEXPR_NUM_THREADS"] = "1"

from rclpy.parameter import Parameter
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from vision_msgs.msg import ObjectHypothesisWithPose
from vision_msgs.msg import Detection2D, Detection2DArray

import cv2
import time
from ultralytics import YOLO


class CameraProcessor(Node):
    def __init__(self):
        super().__init__('camera_processor')

        self.declare_parameter('image_topic', 'image')
        self.declare_parameter('detection_topic', 'image_proc/dets')
        self.declare_parameter('out_image_topic', 'image_proc/result')
        self.declare_parameter('flip_image', False)
        self.declare_parameter('confidence', 0.3)
        self.declare_parameter('model_path', Parameter.Type.STRING)

        image_topic = self.get_parameter(
            'image_topic').get_parameter_value().string_value
        detection_topic = self.get_parameter(
            'detection_topic').get_parameter_value().string_value
        out_image_topic = self.get_parameter(
            'out_image_topic').get_parameter_value().string_value
        self.flip_image = self.get_parameter(
            'flip_image').get_parameter_value().bool_value
        self.confidence = self.get_parameter(
            'confidence').get_parameter_value().double_value
        model_path = self.get_parameter(
            'model_path').get_parameter_value().string_value

        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.camera_callback,
            1)
        self.subscription  # prevent unused variable warning

        # Create Publisher to output annotated results
        self.result_pub_ = self.create_publisher(Image, out_image_topic, 1)
        self.det_pub_ = self.create_publisher(
            Detection2DArray, detection_topic, 5)

        self.br = CvBridge()
        self.model = YOLO(model_path)

        self.frame_count = 0
        self.get_logger().info('Vision Tracker created.')

    def camera_callback(self, msg):
        """
        Callback function.
        """
        t1 = time.clock_gettime(time.CLOCK_THREAD_CPUTIME_ID)

        current_frame = self.br.imgmsg_to_cv2(msg)
        if self.flip_image is True:
            current_frame = cv2.flip(current_frame, -1)

        results, out_img = self.get_detections(current_frame)

        track_msg_arr = self.createDetection2DArr(results, msg.header)
        self.det_pub_.publish(track_msg_arr)

        out_msg = self.br.cv2_to_imgmsg(out_img)
        out_msg.header = msg.header
        self.result_pub_.publish(out_msg)

        self.frame_count += 1
        t2 = time.clock_gettime(time.CLOCK_THREAD_CPUTIME_ID)
        self.get_logger().info(
            f'Frame count: {self.frame_count}, Time (msec): {(t2-t1)*1000:.1f}')

    def get_detections(self, current_frame):
        results = self.model.predict(
            current_frame, save=False, imgsz=(640), conf=self.confidence, device='0', classes=[0, 1, 2, 3, 5, 7], verbose=False)
        # Whoever designed the output of yolov8 made it so complicated
        results = results[0]
        boxes = results.boxes

        if boxes.shape[0] == 0:
            return [None], current_frame
        labels = results.names

        # Draw the bounding boxes and labels on the image
        detections = []
        for box in boxes:
            # Get the coordinates and dimensions of the box
            x1, y1, x2, y2, score, cls = box.data[0].cpu().numpy()
            x1 = int(x1)
            y1 = int(y1)
            x2 = int(x2)
            y2 = int(y2)
            w = x2 - x1
            h = y2 - y1
            detections.append([x1, y1, x2, y2, score, cls])

            # Get the label and color for the box
            label = labels[int(cls)]
            color = (0, 255, 0)  # green

            # Draw the box and the label on the image
            cv2.rectangle(current_frame, (x1, y1), (x1 + w, y1 + h), color, 2)
            cv2.putText(current_frame, label, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        return detections, current_frame

    @staticmethod
    def createDetection2DArr(results, header) -> Detection2DArray:
        """Convert tracker output to message for publishing

        Args:
            tracks (ndarray): Array of the form [[x1,y1,x2,y2,id], [x1,y1,x2,y2,id], ...]

        Returns:
            Detection2DArray
        """
        out = Detection2DArray()
        out.header = header

        for res in results:
            if res is None:
                continue
            det = Detection2D()
            result = ObjectHypothesisWithPose()
            result.hypothesis.score = float(res[4])
            result.hypothesis.class_id = str(res[5])
            det.results.append(result)

            x_len = float(res[2] - res[0])
            y_len = float(res[3] - res[1])
            det.bbox.center.x = x_len/2.0 + res[0]
            det.bbox.center.y = y_len/2.0 + res[1]
            det.bbox.size_x = x_len
            det.bbox.size_y = y_len
            out.detections.append(det)
        return out

def main(args=None):
    rclpy.init(args=args)
    node = CameraProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
