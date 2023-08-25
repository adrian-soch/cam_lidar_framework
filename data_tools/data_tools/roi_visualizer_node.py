#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from visualization_msgs.msg import Marker

import math


class ParamMarkerNode(Node):
    def __init__(self):
        super().__init__('param_marker_node', namespace='lidar_proc')
        self.refresh_period = 0.25
        self.marker_pub = self.create_publisher(Marker, 'marker', 10)

        self.declare_parameter('frame_id', 'map')

        # Height, width, length [meters]
        self.declare_parameter('marker_size', [1.0, 1.0, 1.0])

        # X, Y, X [meters]
        self.declare_parameter('marker_position',
                               [0.0, 0.0, 0.0])

        # R, P, Y (degrees)
        self.declare_parameter('marker_orientation',
                               [0.0, 0.0, 0.0])
        self.timer = self.create_timer(
            self.refresh_period, self.timer_callback)
        self.get_logger().info('ParamMarkerNode initialized')

    def timer_callback(self):

        # get the updated parameters from the node
        frame_id_param = self.get_parameter('frame_id')
        marker_orientation_param = self.get_parameter('marker_orientation')
        marker_size_param = self.get_parameter('marker_size')
        marker_position_param = self.get_parameter('marker_position')

        q = quaternion_from_euler(
            marker_orientation_param.value[0], marker_orientation_param.value[1], marker_orientation_param.value[2])

        marker = Marker()
        marker.header.frame_id = frame_id_param.value
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.id = 0
        marker.action = Marker.ADD

        # set the marker type, color, size and position according to the parameters
        marker.type = Marker.CUBE
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.5
        marker.color.a = 0.4
        marker.scale.x = marker_size_param.value[0]
        marker.scale.y = marker_size_param.value[1]
        marker.scale.z = marker_size_param.value[2]
        marker.pose.position.x = marker_position_param.value[0]
        marker.pose.position.y = marker_position_param.value[1]
        marker.pose.position.z = marker_position_param.value[2]
        marker.pose.orientation.x = q[1]
        marker.pose.orientation.y = q[2]
        marker.pose.orientation.z = q[3]
        marker.pose.orientation.w = q[0]
        marker.lifetime = Duration(seconds=self.refresh_period).to_msg()

        # publish the marker message
        self.marker_pub.publish(marker)


def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [w, x, y, z]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr

    return q


def main(args=None):
    rclpy.init(args=args)
    node = ParamMarkerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
