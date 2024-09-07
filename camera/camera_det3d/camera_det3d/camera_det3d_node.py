"""
@file camera_det3d_node.py

@brief This node subscribes to images and perfroms 3D object detection.
    It publishes detection3D array and Markers for Rviz.

    Reference: W. Zimmer et al., “InfraDet3D: Multi-Modal 3D Object Detection based on Roadside Infrastructure Camera and LiDAR Sensors.”
        arXiv, Apr. 29, 2023. doi: 10.48550/arXiv.2305.00314.

NOTE This should be all C++.


@section Author(s)
- Created by Adrian Sochaniwsky on 2/12/2023
"""
# fmt: off
# Limit CPU use
import os
os.environ["OMP_NUM_THREADS"] = "2"
os.environ["OPENBLAS_NUM_THREADS"] = "2"
os.environ["MKL_NUM_THREADS"] = "2"
os.environ["VECLIB_MAXIMUM_THREADS"] = "2"
os.environ["NUMEXPR_NUM_THREADS"] = "2"


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
# fmt: on


class CameraDet3DNode(Node):
    def __init__(self):
        super().__init__('camera_det3d_node')

        self.intrinsic_matrix, self.rotation_matrix, self.translation, self.c2l_trans = self.init_params()
        self.inv_intrinsic_matrix = np.linalg.inv(self.intrinsic_matrix)

        # Hardcode translation becuase we are only using rotation matrix and not full transforms
        # This means that datasets where camera is not close to lidar we need to shift the data
        # This could be fixed by using full rotation + translation matricies and homogenous coordinates
        self.c2l_trans = [0.0, -13.0, 0.0]

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

        # Must be same frame as LiDAR detections
        self.frame_id = 'map'

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

        return intrinsic_matrix, rotation_matrix, l2g_translation, lidar2cam_trans

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
        det_array.header.frame_id = self.frame_id
        bbox_array = MarkerArray()

        proj_points = np.array([[0, 0, 0]])
        masks = result.masks
        bboxes_2d = result.boxes

        if masks is None or bboxes_2d is None:
            return
        for idx, (mask, bbox_2d) in enumerate(zip(masks, bboxes_2d)):
            obj_class = bbox_2d.cls[0].cpu().numpy()
            bbox_2d = bbox_2d.xyxy[0].cpu().numpy()
            polygon = mask.xy[0]
            # self.draw.polygon(polygon, outline=(0, 255, 0), width=2)

            # Project camera outline to 3D space
            mask3d = self.project_to_ground(polygon.T)

            object_height = self.get_obj_height_estimate(obj_class)
            mask3d = self.refine_3d_countours(
                contour=mask3d, bbox_2d=bbox_2d, object_height=object_height)

            if mask3d.shape[0] == 0:
                continue

            mask3d = self.remove_noise_with_dbscan(
                mask3d, eps=0.65, min_samples=2)
            # Denoising may remove all points from a countour
            if mask3d.shape[0] <= 1:
                continue

            # Get bounding box with L-shape fitting on cleaned points
            bbox_3d = self.l_shape_fit.fitting(mask3d[:, :2])
            bbox_3d.size[2] = object_height

            if 0.0 in bbox_3d.size:
                continue

            # Refine 3D BBox based on 2D BBox
            self.print_3d_to_image(bbox_3d=bbox_3d, bbox_2d=bbox_2d)

            det_array.detections.append(
                self.create_3D_det(header=msg.header, bbox=bbox_3d, obj_cls=obj_class))
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
        self.publish_pc(self.publisher_pcd2, proj_points, 'map')
        self.publish_image(self.img_publisher_, self.pil_img)
        self.bbox_publisher.publish(bbox_array)
        self.det_publisher.publish(det_array)

        end = time.clock_gettime(time.CLOCK_THREAD_CPUTIME_ID)
        self.get_logger().info(
            f'Time (msec): conv {(t1-start)*1000:.1f} inf {(t2-t1)*1000:.1f} proc {(t3-t2)*1000:.1f} pub {(end-t3)*1000:.1f} total {(end-start)*1000:.1f}')

    def refine_3d_countours(self, contour: np.ndarray, bbox_2d, object_height=0.0) -> np.ndarray:
        """Refine the contour coordinate to increase final 3D bbox accuracy

        Args:
            contour (np.ndarray): Array of 2D contour points of the detection
            bbox_2d (np.ndarray): Bbox dimensions in the form [x1,y1,x2,y2] units are pixels.
            object_height (float, optional): Estimated object height in meters, refinement depends on this value,
                if 0.0, all points are retained. Defaults to 0.0.

        Returns:
            np.ndarray: Refined points in 3D coordiantes
        """
        # Get 2D projection from 3D contour
        p2d = self.project_to_image(contour, height=object_height)

        x_offset, y_offset = 0, 2
        x1, y1, x2, y2 = bbox_2d[0]-x_offset, bbox_2d[1] - \
            y_offset, bbox_2d[2]+x_offset, bbox_2d[3]+y_offset

        # check if re-projected point lies inside 2D BBox
        idx_inside_points = (p2d[:, 0] >= x1) & (
            p2d[:, 0] <= x2) & (p2d[:, 1] >= y1) & (p2d[:, 1] <= y2)

        # Get list of points that reproject inside the 2D BBox
        p2d_refined = p2d[idx_inside_points]

        # Convert back to 3D and return
        return self.project_to_ground(p2d_refined.T, ground_plane_height=object_height)

    @staticmethod
    def get_obj_height_estimate(obj_class) -> float:
        #  0: 'person',
        #  1: 'bicycle',
        #  2: 'car',
        #  3: 'motorcycle',
        #  5: 'bus',
        #  7: 'truck',
        height_dict = {'0': 2.0, '1': 1.4, '2': 1.35,
                       '3': 1.25, '5': 2.5, '7': 2.69}

        height = height_dict.get(str(int(obj_class)))
        if height is None:
            height = 1.7

        return height

    def print_3d_to_image(self, bbox_3d, bbox_2d) -> RectangleData:
        """Print 3D Bbox to image for visualization.

        Args:
            bbox_3d (RectangleData): L Fitting result.
            bbox_2d (list): xyxy format of yolov8 bbox

        Returns:
            RectangleData: Refined bbox_3d
        """

        # Prepare the 3D points
        x = np.array(bbox_3d.rect_c_x)
        y = np.array(bbox_3d.rect_c_y)
        p3d = np.vstack([x, y]).T

        p3d = np.vstack([
            np.column_stack([p3d, np.zeros(p3d.shape[0])]),
            np.column_stack([p3d, np.ones(p3d.shape[0])*bbox_3d.size[2]])
        ])

        p2d = self.project_to_image(p3d)

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

        x_offset, y_offset = 1, 1
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

    def project_to_image(self, points_3d: np.ndarray, height=0.0) -> np.ndarray:
        """Project 3D points to 2D image plane.

        Args:
            points_3d (np.ndarray): _description_
            height (float, optional): Height above the ground plane to shift the points in meters. Defaults to 0.0.

        Returns:
            np.ndarray: 2xn array in pixel coordinates
        """
        # First translate
        points_3d -= (self.translation -
                      np.array([0.0, 0.0, height]) + np.array(self.c2l_trans))

        # Tranform to pixel coordinates
        points_2d = np.array(self.intrinsic_matrix @
                             self.rotation_matrix @ points_3d.T)

        # Divide by Z to get 2D projection
        points_2d = np.array((points_2d[:2, :] / points_2d[2:]).T)

        return points_2d

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
        scalar = -(self.translation[2] -
                   ground_plane_height) / ground_points[2]
        ground_points = ground_points.T * \
            scalar[:, np.newaxis] + \
            (self.translation + np.array(self.c2l_trans))
        return ground_points

    def publish_image(self, publisher, image_array) -> None:
        msg = self.br.cv2_to_imgmsg(np.array(image_array))
        publisher.publish(msg)

    @staticmethod
    def publish_pc(publisher, cloud, frame_id: str) -> None:
        pc = {'xyz': cloud}
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
    def create_3D_det(header, bbox, obj_cls=-1) -> Detection3D:
        det = Detection3D()
        det.header = header
        det.id = str(int(obj_cls))
        det.bbox.size.x = float(bbox.size[0])
        det.bbox.size.y = float(bbox.size[1])
        det.bbox.size.z = float(bbox.size[2])

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
