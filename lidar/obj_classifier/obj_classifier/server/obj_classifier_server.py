#! /usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from pipeline_interfaces.srv import Classifier
from joblib import load

import time


class Obj_Classifier(Node):

    def __init__(self):
        super().__init__('Obj_Classifier')
        # Create a service that takes a string as input and returns a string as output
        self.srv = self.create_service(
            Classifier, 'svm_inference', self.svm_inference_callback)

        model_path = self.declare_parameter(
            'model_path', '/home/adrian/dev/A9_images_and_points/a9_dataset_r02_s03').get_parameter_value().string_value

        self.scaler = load(model_path + '/scaler.joblib')
        self.pca = load(model_path + '/pca.joblib')
        self.svm = load(model_path + '/svm.joblib')

    def svm_inference_callback(self, request, response):

        start = time.time()

        feature_vector = np.array(request.request).reshape(1, -1)

        feature_vector = self.scaler.transform(feature_vector)
        feature_vector = self.pca.transform(feature_vector)

        # Predict the class of the feature vector using the SVM model
        predicted_class = self.svm.predict(feature_vector)

        # Set the response data to the predicted class
        response.result = int(predicted_class)

        end = time.time()
        self.get_logger().info(f'Class is {predicted_class}. Time {end-start}')

        return response


def main(args=None):
    rclpy.init(args=args)
    obj_classifier = Obj_Classifier()
    rclpy.spin(obj_classifier)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
