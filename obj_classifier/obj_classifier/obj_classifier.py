#! /usr/bin/env python3
"""
 * @file obj_classifier.py
 * @brief Perform lidar object classification
 * @author Adrian Sochaniwsky (sochania@mcmaster.ca)
"""

import numpy as np
from joblib import load
import time

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection3DArray
from message_filters import ApproximateTimeSynchronizer, Subscriber

from pipeline_interfaces.msg import PointCloud2Array


class Obj_Classifier(Node):

    def __init__(self):
        super().__init__('Obj_Classifier')

        # Get the topic name from the ROS parameter server
        self.world_frame = self.declare_parameter(
            'world_frame', 'map').get_parameter_value().string_value

        detection_topic = self.declare_parameter(
            'detection_topic', '/lidar_proc/o_detections').get_parameter_value().string_value

        model_path = self.declare_parameter(
            'model_path', '/home/adrian/dev/A9_images_and_points/a9_dataset_r02_s03').get_parameter_value().string_value

        self.detection_sub = Subscriber(
            self, Detection3DArray, detection_topic)
        self.pointcloud_sub = Subscriber(
            self, PointCloud2Array, 'lidar_proc/obj_clouds')
        self.sync = ApproximateTimeSynchronizer(
            [self.detection_sub, self.pointcloud_sub], queue_size=3, slop=0.03)
        self.sync.registerCallback(self.callback)

        self.det3d_pub = self.create_publisher(
            Detection3DArray, 'lidar_proc/detections_with_class', 5)

        # Load model params
        self.scaler = load(model_path + '/scaler.joblib')
        self.pca = load(model_path + '/pca.joblib')
        self.svm = load(model_path + '/svm.joblib')

        self.classes = ['BICYCLE', 'BUS', 'CAR', 'EMERGENCY_VEHICLE',
                   'MOTORCYCLE', 'PEDESTRIAN', 'TRAILER', 'TRUCK', 'VAN']

    def callback(self, detections, pointclouds):
        start = time.time()

        feature_vectors = np.empty((len(detections.detections), 7))
        for i, (det, pc) in enumerate(zip(detections.detections, pointclouds.pointclouds)):
            num_points = pc.row_step*pc.height
            length = det.bbox.size.x
            width = det.bbox.size.y
            height = det.bbox.size.z

            x = det.bbox.center.position.x
            y = det.bbox.center.position.y
            z = det.bbox.center.position.z
            dist2sensor = np.linalg.norm([x,y,z])

            w_h_ratio = width/height
            l_h_ratio = length/height

            feature_vectors[i, :] = [num_points, length, width, height, dist2sensor, w_h_ratio, l_h_ratio]

        feature_vectors = self.scaler.transform(feature_vectors)
        feature_vectors = self.pca.transform(feature_vectors)

        # Predict the class of the feature vector using the SVM model
        predicted_classes = self.svm.predict(feature_vectors)

        """
        TODO Unknown classes or objects we dont want should be set to "DONOTTRACK"
        """
        for i, det in enumerate(detections.detections):
            det.id = self.classes[int(predicted_classes[i])]

        self.det3d_pub.publish(detections)

        end = time.time()
        self.get_logger().info(f'Time (msec): {(end-start)*1000:.1f}')


def main(args=None):
    rclpy.init(args=args)
    obj_classifier = Obj_Classifier()
    rclpy.spin(obj_classifier)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
