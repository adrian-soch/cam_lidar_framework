#! /usr/bin/env python3
"""
@file camera_processing_node.py

@brief This node subsribes to images and runs detection+tracking

@section Author(s)
- Created by Adrian Sochaniwsky on 13/11/2022
"""

from pathlib import Path
from cv_bridge import CvBridge
from ultralytics import YOLO
from vision_msgs.msg import Detection2D, Detection2DArray
from vision_msgs.msg import ObjectHypothesisWithPose
from sensor_msgs.msg import Image
from rclpy.node import Node
from camera_processing import vision_track
import rclpy
import time
import json
from datetime import datetime
import cv2
import os
# limit the number of cpus used by high performance libraries
os.environ["OMP_NUM_THREADS"] = "1"
os.environ["OPENBLAS_NUM_THREADS"] = "1"
os.environ["MKL_NUM_THREADS"] = "1"
os.environ["VECLIB_MAXIMUM_THREADS"] = "1"
os.environ["NUMEXPR_NUM_THREADS"] = "1"

# import numpy as np


SAVE_COCO_JSON = False
JSON_PATH = '/home/adrian/dev/metrics/COCO_DATA/'
USE_YOLOV8 = False

if USE_YOLOV8:
    FILE = Path(__file__).resolve()
    ROOT = FILE.parents[0]
    WEIGHTS = ROOT / 'camera_processing' / 'weights'


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
            1   )
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Image subscriber created.')

        # Create Publisher to output annotated results
        self.result_pub_ = self.create_publisher(Image, 'image_proc/result', 5)
        self.track_pub_ = self.create_publisher(
            Detection2DArray, 'image_proc/tracks', 5)

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

        self.frame_count = 0

        if SAVE_COCO_JSON:
            self.json_data = []
            self.dt = datetime.now().strftime("%Y-%m-%dT%H:%M:%S")

        if USE_YOLOV8:
            self.model = YOLO(os.path.join(WEIGHTS, 'yolov8m.engine'))
        else:
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

        if USE_YOLOV8:
            tracks, out_img = self.get_detections(current_frame)
        else:
            tracks, out_img = self.tracker.update(
                current_frame, return_image=True)

        if SAVE_COCO_JSON:
            self.save_to_coco(tracks)

        # Pub track results
        track_msg_arr = self.createDetection2DArr(tracks, msg.header)
        self.track_pub_.publish(track_msg_arr)

        # Pub visual results
        out_msg = self.br.cv2_to_imgmsg(out_img)
        out_msg.header = msg.header
        self.result_pub_.publish(out_msg)

        self.frame_count += 1

        t5 = time.clock_gettime(time.CLOCK_THREAD_CPUTIME_ID)
        self.get_logger().info(
            f'Frame count: {self.frame_count}, Time (msec): {(t5-t1)*1000:.1f}')

    def get_detections(self, current_frame):
        results = self.model.predict(
            current_frame, save=False, imgsz=(640), conf=0.3, device='0', classes=[0, 1, 2, 3, 5, 7], verbose=False)

        # Whoever designed the output of yolov8 made it so complicated
        results = results[0]
        boxes = results.boxes

        if boxes.shape[0] == 0:
            return [None], current_frame
        bbox = boxes.xyxy[0].cpu().numpy()
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
            result.hypothesis.score = float(trk[4])
            result.hypothesis.class_id = str(trk[5])
            det.results.append(result)

            x_len = float(trk[2] - trk[0])
            y_len = float(trk[3] - trk[1])

            det.bbox.center.x = x_len/2.0 + trk[0]
            det.bbox.center.y = y_len/2.0 + trk[1]
            det.bbox.size_x = x_len
            det.bbox.size_y = y_len

            out.detections.append(det)

        return out

    def save_to_coco(self, tracks):
        """Convert track data to COCO json format
        """
        if tracks is None:
            return
        
        for det in tracks:
            if det is None:
                continue
            width, height = float(det[2] - det[0]), float(det[3] - det[1])
            self.json_data.append({"image_id": self.frame_count,
                                            "category_id": int(det[5]),
                                            "bbox": [float(det[0]), float(det[1]),  # x1, y1
                                                    width, height],
                                            "score": float(det[4])}
                                            )

        with open(f'{JSON_PATH}/{self.dt}_COCO.json', 'w', encoding='utf-8') as f:
            json.dump(self.json_data, f, ensure_ascii=False, indent=4)


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
