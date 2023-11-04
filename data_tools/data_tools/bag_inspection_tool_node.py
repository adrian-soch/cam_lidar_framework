#!/usr/bin/env python3

'''
Print the timestamps from the image and points topics. This will help understand the syncronization between the frames

And which frames can be used for producing csv output for computing metrics

'''

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image, PointCloud2
import csv
import os
import time
import uuid

class MyNode(Node):

    def __init__(self):
        super().__init__('my_node')
        self.image_sub = self.create_subscription(Image, 'image', self.image_callback, 10)
        self.points_sub = self.create_subscription(PointCloud2, 'points', self.points_callback, 10)
        # get the output folder from the ros parameter
        default_folder = '/home/adrian/dev/ros2_ws'
        self.output_folder = self.declare_parameter(
            'output_folder', default_folder).get_parameter_value().string_value
        # create the folder if it does not exist
        os.makedirs(self.output_folder, exist_ok=True)
        # generate a unique file name for each node using a timestamp and a random string
        file_name = f'timestamps_{time.time()}_{uuid.uuid4()}.csv'
        # open the csv file in the output folder using the with statement
        with open(os.path.join(self.output_folder, file_name), 'w') as self.csv_file:
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(['image', 'points'])

    def image_callback(self, msg):
        # do something with the image
        self.get_logger().info(f'Received image at {msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}')
        # write the image timestamp to the csv file using the with statement
        with open(self.csv_file.name, 'a') as f:
            csv_writer = csv.writer(f)
            csv_writer.writerow([f'{msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}', ''])

    def points_callback(self, msg):
        # do something with the pointcloud
        self.get_logger().info(f'Received pointcloud at {msg.header.stamp.sec:}.{msg.header.stamp.nanosec:09d}')
        # write the pointcloud timestamp to the csv file using the with statement
        with open(self.csv_file.name, 'a') as f:
            csv_writer = csv.writer(f)
            csv_writer.writerow(['', f'{msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}'])

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()