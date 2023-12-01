import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
import numpy as np
import cv2
import open3d as o3d
import ros2_numpy
from numpy.lib.recfunctions import unstructured_to_structured
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation as R

from ultralytics import YOLO

from PIL import Image as Im
from PIL import ImageDraw


class ImagePCDNode(Node):

    def __init__(self):
        super().__init__('image_pcd_node')

        # CAM_CAL_PATH = '/home/adrian/dev/ros2_ws/src/cam_lidar_tools/camera_pipeline/config/1920x1080_ost.yaml'
        # TRANSFORM_PATH = '/home/adrian/dev/ros2_ws/src/cam_lidar_tools/lidar_pipeline/configs/may10_config.yaml'
        self.IMG_PATH = '/home/adrian/dev/bags/cleaned_bags/may10_q7_rebag/images/000064_1683739424_096948868.jpg'
        PCD_PATH = '/home/adrian/dev/bags/cleaned_bags/may10_q7_rebag/pcds/000064_1683739424_106343845.pcd'

        ls2ld = np.array([[-1.0, 0.0, 0.0],
                          [0.0, -1.0, 0.0],
                          [0.0, 0.0, 1.0]])

        c2l = np.array([[0.0, 1.0, 0.0],
                        [0.0, 0.0, -1.0],
                        [-1.0, 0.0, 0.0]])
        l2c = c2l.T

        l2g_quat = [0.0, 0.2010779, 0.0, 0.9795752]
        self.l2g = R.from_quat(l2g_quat).as_matrix()

        self.rotation_matrix = self.l2g @ c2l @ ls2ld
        self.translation = np.array([0.0, 0.0, 12.0])

        # HD mat
        # self.intrinsic_matrix = [[1199.821557, 0.000000, 960.562236],
        #                          [0.000000, 1198.033465, 551.675808],
        #                          [0.000000, 0.000000, 1.000000]]

        # 1280 x 720 mat
        self.intrinsic_matrix = [[763.836928, 0.000000, 635.352226],
                                 [0.000000, 743.109955, 357.577260],
                                 [0.000000, 0.000000, 1.000000]]
        self.inv_intrinsic_matrix = np.linalg.inv(self.intrinsic_matrix)

        self.img_publisher_ = self.create_publisher(Image, 'image2', 2)
        self.publisher_pcd = self.create_publisher(PointCloud2, 'base', 2)
        self.publisher_pcd2 = self.create_publisher(
            PointCloud2, 'test_det3d', 2)
        self.img = cv2.imread(self.IMG_PATH)
        self.pcd = o3d.io.read_point_cloud(PCD_PATH)
        self.points = np.asarray(self.pcd.points)

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

        self.model = YOLO("yolov8m-seg.pt")

        rate = 1  # Hz
        self.timer = self.create_timer(1.0/rate, self.timer_callback)

        self.draw_flag = False

    def timer_callback(self):
        '''
        TODO scale the image myself, retain the ratio to scale the points back so the
        intrinsic matrix is still valid
        '''

        # Rough point filtering
        pc = self.points
        pc = pc[np.logical_not(pc[:, 0] <= 0)]

        pc = self.l2g @ pc.T
        pc = pc.T + self.translation

        # Get seg masks
        results = self.model.predict(
            self.img, save=False, imgsz=(1280, 720), conf=0.5, device='0')
        result = results[0]

        # TODO make this a forloop for all masks
        masks = result.masks
        mask1 = masks[0]

        mask = mask1.data[0].cpu().numpy()
        polygon = mask1.xy[0]

        mask3d = self.project_to_ground(polygon.T)

        # Create a PointCloud2 message from the numpy array
        pc = unstructured_to_structured(pc, dtype=np.dtype(
            [('x', '<f4'), ('y', '<f4'), ('z', '<f4')]))
        msg = ros2_numpy.msgify(PointCloud2, pc)
        msg.header.frame_id = 'map'
        self.publisher_pcd.publish(msg)

        pc2 = unstructured_to_structured(mask3d, dtype=np.dtype(
            [('x', '<f4'), ('y', '<f4'), ('z', '<f4')]))
        msg2 = ros2_numpy.msgify(PointCloud2, pc2)
        msg2.header.frame_id = 'map'
        self.publisher_pcd2.publish(msg2)
        self.get_logger().info('Publishing point cloud')

        msg3 = self.br.cv2_to_imgmsg(self.img)
        self.img_publisher_.publish(msg3)

        if self.draw_flag == False:
            img = Im.open(self.IMG_PATH)
            draw = ImageDraw.Draw(img)
            draw.polygon(polygon, outline=(0, 255, 0), width=5)
            img.show()
            self.draw_flag = True

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


def main(args=None):
    rclpy.init(args=args)
    node = ImagePCDNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
