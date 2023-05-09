"""
This node subscribes to a Detection3DArray which storestracker results
It will publish the tracking results as marker array to be visualized in rviz
"""
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from vision_msgs.msg import Detection3DArray, Detection3D
from visualization_msgs.msg import Marker, MarkerArray

import random

class DetectorNode(Node):
    def __init__(self):
        super().__init__('detector_node')

        # Get the topic name from the ROS parameter server
        topic_name = self.declare_parameter('topic_name', '/lidar_proc/tracks').get_parameter_value().string_value

        # Subscribe to the detection messages
        self.subscription = self.create_subscription(
            Detection3DArray, topic_name, self.detection_callback, 5)

        # Publish the marker messages
        self.bbox_publisher = self.create_publisher(MarkerArray, 'lidar_proc/tracker_bboxs', 5)
        self.tracklet_publisher = self.create_publisher(MarkerArray, '/lidar_proc/tracklets', 5)

        self.get_logger().info('Starting Tracker BBox Visualization')

    def detection_callback(self, msg):
        # Create a MarkerArray message
        track_bbox_array = MarkerArray()
        tracklet_array = MarkerArray()

        # Loop through each detection
        idx = 0
        for detection in msg.detections:

            # Get track ID
            trk_id = detection.results[0].score

            # Get a colour based on the track ID
            # set the seed value so the same colour is applied
            # to the same track each time
            random.seed(trk_id) 
            r = random.random()
            g = random.random()
            b = random.random()

            # Get the position and size of the detection
            pos = detection.bbox.center.position
            size = detection.bbox.size

            # Create a cube marker for the detection
            marker = Marker()
            marker.id = idx
            marker.header = msg.header
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position = pos
            marker.scale.x = size.x
            marker.scale.y = size.y
            marker.scale.z = 0.25
            marker.color.a = 0.6
            marker.color.r = r
            marker.color.g = g
            marker.color.b = b
            marker.lifetime = Duration(seconds=0.1).to_msg()
            
            # Create a sphere marker for the track
            tracklet = Marker()

            # TODO fix this id so we dont overwrite any markers
            tracklet.id = idx
            tracklet.header = msg.header
            tracklet.type = Marker.sphere
            tracklet.action = Marker.ADD
            tracklet.pose.position = pos
            tracklet.scale.x = 0.3
            tracklet.scale.y = 0.3
            tracklet.scale.z = 0.3
            tracklet.color.a = 0.6
            tracklet.color.r = r
            tracklet.color.g = g
            tracklet.color.b = b
            tracklet.lifetime = Duration(seconds=0.5).to_msg()

            # Add the marker to the array
            idx += 1
            track_bbox_array.markers.append(marker)
            tracklet_array.markers.append(tracklet)

        # Publish the marker array
        self.bbox_publisher.publish(track_bbox_array)
        self.tracklet_publisher.publish(tracklet_array)


def main(args=None):
    rclpy.init(args=args)
    detector_node = DetectorNode()
    rclpy.spin(detector_node)
    detector_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
