import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge

import numpy as np
import cv2
import open3d as o3d
import ros2_numpy
from numpy.lib.recfunctions import unstructured_to_structured
from scipy.spatial.transform import Rotation as R
from sklearn.cluster import DBSCAN

from ultralytics import YOLO

from PIL import Image as Im
from PIL import ImageDraw

from l_shape import LShapeFitting


class ImagePCDNode(Node):

    def __init__(self):
        super().__init__('image_pcd_node')

        self.IMG_PATH = '/home/adrian/dev/bags/cleaned_bags/may10_q7_rebag/images/000064_1683739424_096948868.jpg'
        PCD_PATH = '/home/adrian/dev/bags/cleaned_bags/may10_q7_rebag/pcds/000064_1683739424_106343845.pcd'

        self.intrinsic_matrix = [[1199.821557, 0.000000, 960.562236],
                                 [0.000000, 1198.033465, 551.675808],
                                 [0.000000, 0.000000, 1.000000]]
        self.inv_intrinsic_matrix = np.linalg.inv(self.intrinsic_matrix)

        self.ls2ld = np.array([[-1.0, 0.0, 0.0],
                               [0.0, -1.0, 0.0],
                               [0.0, 0.0, 1.0]])

        c2l = np.array([[0.0, 1.0, 0.0],
                        [0.0, 0.0, -1.0],
                        [-1.0, 0.0, 0.0]])

        # x,y,z,w
        # l2g_quat = [0.0, 0.2010779, 0.0, 0.9795752]
        # self.l2g = R.from_quat(l2g_quat).as_matrix()

        self.l2g = R.from_euler('xyz', angles=[0,23,0], degrees=True).as_matrix()

        self.rotation_matrix = c2l @ self.ls2ld @ self.l2g.T
        self.translation = np.array([0.0, 0.0, 11.0])

        self.img_publisher_ = self.create_publisher(Image, 'image2', 2)
        self.publisher_pcd = self.create_publisher(PointCloud2, 'base', 2)
        self.publisher_pcd2 = self.create_publisher(
            PointCloud2, 'test_det3d', 2)
        self.bbox_publisher = self.create_publisher(
            MarkerArray, 'cam_bbox3D', 2)
        self.pcd = o3d.io.read_point_cloud(PCD_PATH)
        self.points = np.asarray(self.pcd.points)

        self.model = YOLO("yolov8m-seg.pt")
        self.br = CvBridge()
        self.img = Im.open(self.IMG_PATH)
        self.draw = ImageDraw.Draw(self.img)

        self.l_shape_fit = LShapeFitting()

        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):

        # Rough point filtering
        base_cloud = self.points
        base_cloud = base_cloud[np.logical_not(base_cloud[:, 0] <= 0)]

        base_cloud = self.l2g @ base_cloud.T
        base_cloud = base_cloud.T + [0,0,12]

        '''
        Project Segmentation masks to ground
        '''
        results = self.model.predict(
            self.img, save=False, imgsz=(640), conf=0.5, device='0', classes=[0, 1, 2, 3, 5, 7])
        result = results[0]

        bbox_array = MarkerArray()
        proj_points = np.array([[0, 0, 0]])
        masks = result.masks
        for idx, mask in enumerate(masks):
            polygon = mask.xy[0]
            self.draw.polygon(polygon, outline=(0, 255, 0), width=2)

            # Project camera outline to 3D space
            mask3d = self.project_to_ground(polygon.T)
            mask3d = self.remove_noise_with_dbscan(mask3d, eps=0.75, min_samples=3)

            if mask3d.shape[0] <= 1:
                continue

            # Get Bbox with L-shape fitting on cleaned points
            bbox = self.l_shape_fit.fitting(mask3d[:, :2])
            bbox_array.markers.append(self.createMarker(bbox, idx, 'map'))

            # Debug cloud to see all countour points
            proj_points = np.vstack([proj_points, mask3d])

            # Uncomment to have BBox corner points
            # lshapeCont2d = np.vstack([bbox.rect_c_x, bbox.rect_c_y]).T
            # lshapeCont3D = np.c_[lshapeCont2d, np.zeros(lshapeCont2d.shape[0])]

        '''
        Publish image, base pc, and projected points
        '''
        self.publish_pc(self.publisher_pcd, base_cloud, 'map')
        self.publish_pc(self.publisher_pcd2, proj_points, 'map')
        self.publish_image(self.img_publisher_, self.img)
        self.bbox_publisher.publish(bbox_array)

        self.get_logger().info('Publishing point cloud')
        exit(0)

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
        # marker.lifetime = Duration(seconds=0.1).to_msg()

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
    node = ImagePCDNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
