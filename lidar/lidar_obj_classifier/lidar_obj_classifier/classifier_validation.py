#! /usr/bin/env python3
"""
 * @file classifier_validation.py
 * @brief Perform lidar object classification validation
 * @author Adrian Sochaniwsky (sochania@mcmaster.ca)
"""
import cv2
import numpy as np
from joblib import load
import time

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection3DArray
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import PointCloud2

from pipeline_interfaces.msg import PointCloud2Array

import ros2_numpy


class Obj_Classifier(Node):

    def __init__(self):
        super().__init__('Obj_Classifier')

        # Get the topic name from the ROS parameter server
        self.world_frame = self.declare_parameter(
            'world_frame', 'map').get_parameter_value().string_value

        detection_topic = self.declare_parameter(
            'detection_topic', '/lidar_proc/o_detections').get_parameter_value().string_value

        pointcloud_topic = self.declare_parameter(
            'pointcloud_topic', 'lidar_proc/obj_clouds').get_parameter_value().string_value

        model_path = self.declare_parameter(
            'model_path', '/home/adrian/dev/A9_images_and_points/a9_dataset_r02_s03').get_parameter_value().string_value

        self.detection_sub = Subscriber(
            self, Detection3DArray, detection_topic)
        self.pointcloud_sub = Subscriber(
            self, PointCloud2Array, pointcloud_topic)
        self.sync = ApproximateTimeSynchronizer(
            [self.detection_sub, self.pointcloud_sub], queue_size=3, slop=0.03)
        self.sync.registerCallback(self.callback)

        self.det3d_pub = self.create_publisher(
            Detection3DArray, 'lidar_proc/detections_with_class', 2)

        self.pc_pub = self.create_publisher(
            PointCloud2, 'lidar_proc/isolated_pointcloud', 2)

        # Load model params
        self.scaler = load(model_path + '/scaler.joblib')
        self.pca = load(model_path + '/pca.joblib')
        self.svm = load(model_path + '/svm.joblib')

        self.classes = ['BICYCLE', 'BUS', 'CAR', 'EMERGENCY_VEHICLE',
                        'MOTORCYCLE', 'PEDESTRIAN', 'TRAILER', 'TRUCK', 'VAN']

        # # For use of waitkey function
        # cv2.imshow('frame', None)

        cv2.namedWindow("frame", cv2.WINDOW_NORMAL | cv2.WINDOW_FREERATIO)

    def callback(self, detections, pointclouds):
        start = time.time()

        for i, (det, pc) in enumerate(zip(detections.detections, pointclouds.pointclouds)):
            np_pc = ros2_numpy.numpify(pc)

            num_points = np_pc.size
            length = det.bbox.size.x
            width = det.bbox.size.y
            height = det.bbox.size.z
            x = det.bbox.center.position.x
            y = det.bbox.center.position.y
            z = det.bbox.center.position.z
            dist2sensor = np.linalg.norm([x, y, z])

            w_h_ratio = width/height
            l_h_ratio = length/height

            # feature_vector = [num_points, length, width,
            #                   height, dist2sensor, w_h_ratio, l_h_ratio]

            feature_vector = np.array((num_points, length, width,
                                       height, dist2sensor, w_h_ratio, l_h_ratio)).reshape(1, -1)
            self.get_logger().info(f'Features {int(num_points)}.')

            feature_vector = self.scaler.transform(feature_vector)
            feature_vector = self.pca.transform(feature_vector)

            # Predict the class of the feature vector using the SVM model
            predicted_class = self.svm.predict(feature_vector)

            y_new = self.svm.decision_function(feature_vector)
            self.get_logger().info(f'Class results {y_new}.')

            det.id = self.classes[int(predicted_class)]
            self.get_logger().info(f'Class is {det.id}.')

            pc.header.frame_id = 'map'
            self.pc_pub.publish(pc)

            key = cv2.waitKey(0)
            if key == 13:
                # if enter is pressed continue to next
                pass
            elif key == 27:
                # if escape is pressed close down
                cv2.destroyAllWindows()
                exit(0)

        self.det3d_pub.publish(detections)

        # end = time.time()
        # self.get_logger().info(f'Time (msec): {(end-start)*1000:.1f}')


def main(args=None):
    rclpy.init(args=args)
    lidar_obj_classifier = Obj_Classifier()
    rclpy.spin(lidar_obj_classifier)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
