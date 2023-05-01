"""
This node subscribes to a Detection3DArray which storestracker results
It will publish the tracking results as marker array to be visualized in rviz
"""
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from vision_msgs.msg import Detection3DArray
from visualization_msgs.msg import Marker, MarkerArray

class DetectorNode(Node):
    def __init__(self):
        super().__init__('detector_node')

        # Get the topic name from the ROS parameter server
        topic_name = self.declare_parameter('topic_name', '/lidar_proc/tracks').get_parameter_value().string_value

        # Subscribe to the detection messages
        self.subscription = self.create_subscription(
            Detection3DArray, topic_name, self.detection_callback, 5)

        # Publish the marker messages
        self.publisher = self.create_publisher(MarkerArray, 'lidar_proc/tracker_bbox', 5)

        self.get_logger().info('Starting Tracker BBox Visualization')

    def detection_callback(self, msg):
        # Create a MarkerArray message
        marker_array = MarkerArray()

        # Loop through each detection
        idx = 0
        for detection in msg.detections:
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
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0

            marker.lifetime = Duration(seconds=0.1).to_msg()

            # Add the marker to the array
            idx += 1
            marker_array.markers.append(marker)

        # Publish the marker array
        self.publisher.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    detector_node = DetectorNode()
    rclpy.spin(detector_node)
    detector_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
