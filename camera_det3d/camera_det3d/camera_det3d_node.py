"""
@file camera_det3d_node.py

@brief This node subscribes to images and perfroms 3D object detection.
    It publishes detection3D array and Markers for Rviz.

    Reference: W. Zimmer et al., “InfraDet3D: Multi-Modal 3D Object Detection based on Roadside Infrastructure Camera and LiDAR Sensors.”
        arXiv, Apr. 29, 2023. doi: 10.48550/arXiv.2305.00314.


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
from vision_msgs.msg import Detection3D, Detection3DArray
from visualization_msgs.msg import Marker, MarkerArray

import rclpy
from rclpy.duration import Duration
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.parameter import ParameterType

from camera_det3d.l_shape import LShapeFitting, RectangleData
from ultralytics import YOLO


class CameraDet3DNode(Node):
    def __init__(self):
        super().__init__('camera_det3d_node')

        self.intrinsic_matrix, self.rotation_matrix, self.translation = self.init_params()
        self.inv_intrinsic_matrix = np.linalg.inv(self.intrinsic_matrix)

        self.cam2ground = self.rotation_matrix.T @ self.inv_intrinsic_matrix

        self.img_publisher_ = self.create_publisher(
            Image, '/image_proc/contours', 2)
        self.publisher_pcd2 = self.create_publisher(
            PointCloud2, '/image_proc/test_det3d', 2)
        self.bbox_publisher = self.create_publisher(
            MarkerArray, '/image_proc/cam_bbox3D', 2)
        self.det_publisher = self.create_publisher(
            Detection3DArray, 'image_proc/det3D', 2)
        
        weights = self.declare_parameter(
            'weights', 'yolov8m-seg.pt').get_parameter_value().string_value

        self.model = YOLO(weights)
        self.br = CvBridge()
        self.l_shape_fit = LShapeFitting()

        # Subscribe to the image topic
        self.image_subscription = self.create_subscription(
            Image, 'image', self.image_callback, 5
        )
        self.image_subscription  # prevent unused variable warning

        self.pil_img = None
        self.img = None
        self.draw = None

        self.get_logger().info('Setup complete.')

    def init_params(self):
        """Get all transforms and return final transform

        Returns:
            ndarray, ndarray, ndarray: K, Rot, Trans
        """
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

        self.img = ros2_numpy.numpify(msg)
        self.pil_img = Im.fromarray(self.img)
        self.draw = ImageDraw.Draw(self.pil_img)

        t1 = time.clock_gettime(time.CLOCK_THREAD_CPUTIME_ID)

        results = self.model.predict(
            self.img, save=False, imgsz=(640), conf=0.5, device='0', classes=[0, 1, 2, 3, 5, 7], verbose=False)
        result = results[0]

        t2 = time.clock_gettime(time.CLOCK_THREAD_CPUTIME_ID)

        det_array = Detection3DArray()
        det_array.header = msg.header
        bbox_array = MarkerArray()

        proj_points = np.array([[0, 0, 0]])
        masks = result.masks
        bboxes_2d = result.boxes
        for idx, (mask, bbox_2d) in enumerate(zip(masks, bboxes_2d)):
            l1 = time.clock_gettime(time.CLOCK_THREAD_CPUTIME_ID)

            bbox_2d = bbox_2d.xyxy[0].cpu().numpy()
            polygon = mask.xy[0]
            # self.draw.polygon(polygon, outline=(0, 255, 0), width=2)

            # Project camera outline to 3D space
            mask3d = self.project_to_ground(polygon.T)
            mask3d = self.remove_noise_with_dbscan(
                mask3d, eps=0.65, min_samples=2)
            
            l2 = time.clock_gettime(time.CLOCK_THREAD_CPUTIME_ID)

            # Denoising may remove all points from a countour
            if mask3d.shape[0] <= 1:
                continue

            # Get bounding box with L-shape fitting on cleaned points
            bbox_3d = self.l_shape_fit.fitting(mask3d[:, :2])

            if 0.0 in bbox_3d.size:
                continue

            l3 = time.clock_gettime(time.CLOCK_THREAD_CPUTIME_ID)

            # Refine 3D BBox based on 2D BBox
            bbox_3d = self.refine_3d_dets(bbox_3d=bbox_3d, bbox_2d=bbox_2d)

            det_array.detections.append(
                self.create_3D_det(header=msg.header, bbox=bbox_3d))
            bbox_array.markers.append(self.create_marker(
                bbox=bbox_3d, index=idx, frame_id='map'))

            # Debug cloud to see all countour points
            proj_points = np.vstack([proj_points, mask3d])

            l4 = time.clock_gettime(time.CLOCK_THREAD_CPUTIME_ID)
            # self.get_logger().info(
                # f'Time (msec): proj {(l2-l1)*1000:.2f} lfit {(l3-l2)*1000:.2f} ref {(l4-l3)*1000:.2f} ')

        t3 = time.clock_gettime(time.CLOCK_THREAD_CPUTIME_ID)

        '''
        Publish image, base pc, and projected points
        '''
        # self.publish_pc(self.publisher_pcd2, proj_points, 'map')
        self.publish_image(self.img_publisher_, self.pil_img)
        self.bbox_publisher.publish(bbox_array)
        self.det_publisher.publish(det_array)

        end = time.clock_gettime(time.CLOCK_THREAD_CPUTIME_ID)
        self.get_logger().info(
            f'Time (msec): conv {(t1-start)*1000:.1f} inf {(t2-t1)*1000:.1f} proc {(t3-t2)*1000:.1f} pub {(end-t3)*1000:.1f} total {(end-start)*1000:.1f}')

    def refine_3d_dets(self, bbox_3d, bbox_2d) -> RectangleData:
        """Height Estimation and Dimension Filtering The height for each detection is initialized from a fixed value for the object type of the detection.
           Both the height and the location are then jointly optimized through binary search, until the estimated projected 2D object height and the original
           mask height are the same by < 1px. The length and width values, as estimated by the L-Shape-Fitting algorithm for each 3D bottom contour,
           are limited to minimum and maximum.`

        Args:
            bbox_3d (custom class): L-shape fitting result of projected contour
            bbox_2d (list): xyxy format of yolov8 bbox

        Returns:
            custom class: Refined bbox_3d
        """

        # Prepare the 3D points
        x = np.array(bbox_3d.rect_c_x)
        y = np.array(bbox_3d.rect_c_y)
        p3d = np.vstack([x, y]).T

        p3d = np.vstack([
            np.column_stack([p3d, np.zeros(p3d.shape[0])]),
            np.column_stack([p3d, np.ones(p3d.shape[0])*2.3])
        ])

        # First translate
        p3d -= self.translation

        # Tranform to pixel coordinates
        p2d = np.array(self.intrinsic_matrix @ self.rotation_matrix @ p3d.T)

        # Divide by Z to get 2-D projection
        p2d = np.array((p2d[:2, :] / p2d[2:]).T)

        lines = [
            (p2d[0][0], p2d[0][1], p2d[1][0], p2d[1][1]),
            (p2d[1][0], p2d[1][1], p2d[2][0], p2d[2][1]),
            (p2d[2][0], p2d[2][1], p2d[3][0], p2d[3][1]),
            (p2d[3][0], p2d[3][1], p2d[0][0], p2d[0][1]),

            (p2d[4][0], p2d[4][1], p2d[5][0], p2d[5][1]),
            (p2d[5][0], p2d[5][1], p2d[6][0], p2d[6][1]),
            (p2d[6][0], p2d[6][1], p2d[7][0], p2d[7][1]),
            (p2d[7][0], p2d[7][1], p2d[4][0], p2d[4][1]),

            (p2d[0][0], p2d[0][1], p2d[4][0], p2d[4][1]),
            (p2d[1][0], p2d[1][1], p2d[5][0], p2d[5][1]),
            (p2d[2][0], p2d[2][1], p2d[6][0], p2d[6][1]),
            (p2d[3][0], p2d[3][1], p2d[7][0], p2d[7][1]),
        ]
        # Draw the lines on the image
        for line in lines:
            self.draw.line(line, fill=(255, 0, 0), width=5)

        x_offset, y_offset = 15, 15
        x1, y1, x2, y2 = bbox_2d[0]-x_offset, bbox_2d[1] - \
            y_offset, bbox_2d[2]+x_offset, bbox_2d[3]+y_offset
        lines2 = [
            (x1, y1, x2, y1),
            (x1, y2, x2, y2),
            (x1, y1, x1, y2),
            (x2, y1, x2, y2),
        ]
        for line in lines2:
            self.draw.line(line, fill=(255, 255, 0), width=5)

        return bbox_3d

    def project_to_ground(self, image_points: np.ndarray, ground_plane_height=0) -> np.ndarray:
        """
        Project image points (2xn) into ground frame (3xn) i.e. z=0.
        """
        # "Transform" points into ground frame.
        # The real ground point is somewhere on the line going through the camera position and the respective point.
        augmented_points = np.vstack(
            (image_points, np.ones(image_points.shape[1])))
        ground_points = self.cam2ground @ augmented_points
        # Find intersection of line with ground plane i.e. z=0.
        scalar = -(self.translation[2] - ground_plane_height) / ground_points[2]
        ground_points = ground_points.T * scalar[:, np.newaxis] + self.translation
        return ground_points

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

    @staticmethod
    def create_3D_det(header, bbox):
        det = Detection3D()
        det.header = header
        det.bbox.size.x = float(bbox.size[0])
        det.bbox.size.y = float(bbox.size[1])
        det.bbox.size.z = 2.3

        det.bbox.center.position.x = bbox.center[0]
        det.bbox.center.position.y = bbox.center[1]
        det.bbox.center.position.z = det.bbox.size.z/2.0

        det.bbox.center.orientation.x = bbox.quat[0]
        det.bbox.center.orientation.y = bbox.quat[1]
        det.bbox.center.orientation.z = bbox.quat[2]
        det.bbox.center.orientation.w = bbox.quat[3]

        return det

    @staticmethod
    def create_marker(bbox, index: int, frame_id: str) -> Marker:
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


def main(args=None):
    rclpy.init(args=args)
    camera_det3d_node = CameraDet3DNode()
    rclpy.spin(camera_det3d_node)
    camera_det3d_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
