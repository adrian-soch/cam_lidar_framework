"""
@file camera_det3d_node.py

@brief This node subscribes to images and perfroms 3D object detection.
    It publishes detection3D array and Markers for Rviz.

@section Author(s)
- Created by Adrian Sochaniwsky on 2/12/2023
"""

# Limit CPU use
import os
os.environ["OMP_NUM_THREADS"] = "1"
os.environ["OPENBLAS_NUM_THREADS"] = "1"
os.environ["MKL_NUM_THREADS"] = "1"
os.environ["VECLIB_MAXIMUM_THREADS"] = "1"
os.environ["NUMEXPR_NUM_THREADS"] = "1"


from numpy.lib.recfunctions import unstructured_to_structured
import numpy as np
from PIL import ImageDraw, Image as Im
import time
from sklearn.cluster import DBSCAN
from scipy.spatial.transform import Rotation as R

import ros2_numpy
from cv_bridge import CvBridge
from sensor_msgs.msg import PointCloud2, Image
from visualization_msgs.msg import Marker, MarkerArray

import rclpy
from rclpy.duration import Duration
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.parameter import ParameterType

from camera_det3d.l_shape import LShapeFitting
from ultralytics import YOLO


class CameraDet3DNode(Node):
    def __init__(self):
        super().__init__('camera_det3d_node')

        self.intrinsic_matrix, self.rotation_matrix, self.translation = self.init_params()
        self.inv_intrinsic_matrix = np.linalg.inv(self.intrinsic_matrix)

        self.img_publisher_ = self.create_publisher(
            Image, '/image_proc/contours', 2)
        self.publisher_pcd2 = self.create_publisher(
            PointCloud2, 'test_det3d', 2)
        self.bbox_publisher = self.create_publisher(
            MarkerArray, 'cam_bbox3D', 2)

        # Weights will be downloaded on first run
        self.model = YOLO("yolov8m-seg.pt")
        self.br = CvBridge()
        self.l_shape_fit = LShapeFitting()

        # Subscribe to the image topic
        self.image_subscription = self.create_subscription(
            Image, 'image', self.image_callback, 5
        )
        self.image_subscription  # prevent unused variable warning

        self.get_logger().info('Setup complete.')

    def init_params(self):
        self.declare_parameter('camera_matrix', descriptor=ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE_ARRAY))
        self.declare_parameter('lidar2cam_extrinsic.rotation', descriptor=ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE_ARRAY))
        self.declare_parameter('lidar2cam_extrinsic.translation', descriptor=ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE_ARRAY))
        self.declare_parameter('lidarData2lidarSensor_extrinsic.rotation', descriptor=ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE_ARRAY))
        self.declare_parameter('lidarData2lidarSensor_extrinsic.translation',
                               descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY))
        self.declare_parameter('lidar2world_transform.quaternion', descriptor=ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE_ARRAY))
        self.declare_parameter('lidar2world_transform.translation', descriptor=ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE_ARRAY))

        intrinsic_matrix = self.get_parameter(
            'camera_matrix').get_parameter_value().double_array_value
        intrinsic_matrix = np.array(intrinsic_matrix).reshape([3, 3])

        c2l = self.get_parameter(
            'lidar2cam_extrinsic.rotation').get_parameter_value().double_array_value
        c2l = np.array(c2l).reshape([3, 3])

        lidar2cam_trans = self.get_parameter(
            'lidar2cam_extrinsic.translation').get_parameter_value().double_array_value

        ldata2lsensor = self.get_parameter(
            'lidarData2lidarSensor_extrinsic.rotation').get_parameter_value().double_array_value
        ldata2lsensor = np.array(ldata2lsensor).reshape([3, 3])

        lidarData2lidarSensor_trans = self.get_parameter(
            'lidarData2lidarSensor_extrinsic.translation').get_parameter_value().double_array_value

        l2g_quat = self.get_parameter(
            'lidar2world_transform.quaternion').get_parameter_value().double_array_value

        l2g_translation = self.get_parameter(
            'lidar2world_transform.translation').get_parameter_value().double_array_value

        # x,y,z,w
        l2g = R.from_quat(
            (l2g_quat[1], l2g_quat[2], l2g_quat[3], l2g_quat[0])).as_matrix()

        rotation_matrix = c2l @ ldata2lsensor @ l2g.T

        return intrinsic_matrix, rotation_matrix, l2g_translation

    def image_callback(self, msg):
        start = time.clock_gettime(time.CLOCK_THREAD_CPUTIME_ID)

        img = ros2_numpy.numpify(msg)
        pil_img = Im.fromarray(img)
        draw = ImageDraw.Draw(pil_img)

        t1 = time.clock_gettime(time.CLOCK_THREAD_CPUTIME_ID)

        results = self.model.predict(
            img, save=False, imgsz=(640), conf=0.5, device='0', classes=[0, 1, 2, 3, 5, 7], verbose=False)
        result = results[0]

        t2 = time.clock_gettime(time.CLOCK_THREAD_CPUTIME_ID)

        bbox_array = MarkerArray()
        proj_points = np.array([[0, 0, 0]])
        masks = result.masks
        for idx, mask in enumerate(masks):
            polygon = mask.xy[0]
            # draw.polygon(polygon, outline=(0, 255, 0), width=2)

            # Project camera outline to 3D space
            mask3d = self.project_to_ground(polygon.T)
            mask3d = self.remove_noise_with_dbscan(
                mask3d, eps=0.65, min_samples=3)

            # Denoising may remove all points from a countour
            if mask3d.shape[0] <= 1:
                continue

            # Get Bbox with L-shape fitting on cleaned points
            bbox = self.l_shape_fit.fitting(mask3d[:, :2])
            bbox_array.markers.append(self.createMarker(bbox, idx, 'map'))

            # Debug cloud to see all countour points
            proj_points = np.vstack([proj_points, mask3d])

        t3 = time.clock_gettime(time.CLOCK_THREAD_CPUTIME_ID)

        '''
        Publish image, base pc, and projected points
        '''
        self.publish_pc(self.publisher_pcd2, proj_points, 'map')
        self.publish_image(self.img_publisher_, pil_img)
        self.bbox_publisher.publish(bbox_array)

        end = time.clock_gettime(time.CLOCK_THREAD_CPUTIME_ID)
        self.get_logger().info(
            f'Time (msec): conv {(t1-start)*1000:.1f} inf {(t2-t1)*1000:.1f} proc {(t3-t2)*1000:.1f} pub {(end-t3)*1000:.1f} total {(end-start)*1000:.1f}')

    @staticmethod
    def createMarker(bbox, index: int, frame_id: str) -> Marker:
        marker = Marker()
        marker.id = index
        marker.header.frame_id = frame_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.scale.x = float(bbox.size[0])
        marker.scale.y = float(bbox.size[1])
        marker.scale.z = 2.3

        marker.pose.position.x = float(bbox.center[0])
        marker.pose.position.y = float(bbox.center[1])
        marker.pose.position.z = marker.scale.z/2.0

        marker.pose.orientation.x = bbox.quat[0]
        marker.pose.orientation.y = bbox.quat[1]
        marker.pose.orientation.z = bbox.quat[2]
        marker.pose.orientation.w = bbox.quat[3]

        marker.color.a = 0.5
        marker.color.r = 0.59
        marker.color.g = 0.76
        marker.color.b = 0.1
        marker.lifetime = Duration(seconds=0.1).to_msg()

        return marker

    def publish_image(self, publisher, image_array) -> None:
        msg = self.br.cv2_to_imgmsg(np.array(image_array))
        publisher.publish(msg)

    @staticmethod
    def publish_pc(publisher, cloud, frame_id: str) -> None:
        pc = unstructured_to_structured(cloud, dtype=np.dtype(
            [('x', '<f4'), ('y', '<f4'), ('z', '<f4')]))
        msg = ros2_numpy.msgify(PointCloud2, pc)
        msg.header.frame_id = frame_id
        publisher.publish(msg)

    def project_to_ground(self, image_points: np.ndarray, ground_plane_height=0) -> np.ndarray:
        """Project image points (2xn) into ground frame (3xn) i.e. z=0."""
        assert image_points.shape[0] == 2
        # "Transform" points into ground frame.
        # The real ground point is somewhere on the line going through the camera position and the respective point.
        augmented_points = np.vstack(
            (image_points, np.ones(image_points.shape[1])))
        ground_points = self.rotation_matrix.T @ self.inv_intrinsic_matrix @ augmented_points
        # Find intersection of line with ground plane i.e. z=0.
        ground_points *= - \
            (self.translation[2] - ground_plane_height) / ground_points[2]
        result = ground_points.T + self.translation
        return result

    @staticmethod
    def remove_noise_with_dbscan(data, eps=0.5, min_samples=5) -> np.ndarray:
        """
        Perform DBSCAN clustering to identify and remove noise from 2D data.

        Parameters:
        - data: A 2D numpy array of shape (n_samples, n_features).
        - eps: The maximum distance between two samples for them to be considered as in the same neighborhood.
        - min_samples: The number of samples in a neighborhood for a point to be considered as a core point.

        Returns:
        - core_samples: The 2D numpy array of core samples without noise.
        """
        # Initialize DBSCAN
        dbscan = DBSCAN(eps=eps, min_samples=min_samples)

        # Fit the model and get the labels
        labels = dbscan.fit_predict(data)

        # Identify core samples (non-noise points have labels >= 0)
        core_samples_mask = labels >= 0

        # Extract core samples
        core_samples = data[core_samples_mask]

        return core_samples


def main(args=None):
    rclpy.init(args=args)
    camera_det3d_node = CameraDet3DNode()
    rclpy.spin(camera_det3d_node)
    camera_det3d_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
