'''
This script converts the A9 data into BEV psuedo images
based on the config file values
'''

import argparse
import cv2
import os
import open3d as o3d
import numpy as np
from pypcd4 import PointCloud
from torch.utils.data import Dataset

import configs.a9_config as cfg
from lidar_2_bev import get_gt, radius_outlier_removal, transform_pc, array_to_image, get_files, euler_from_quaternion, shuffle_list


class A9LidarBevCreator(Dataset):
    def __init__(self, input_path, output_path):
        # store the input and output folder paths as attributes
        input_path = input_path
        self.output_path = output_path

        print('Getting files from path.')
        sensor_directions = [
            's110_lidar_ouster_north', 's110_lidar_ouster_south']
        self.lidar_list = []
        for dir in sensor_directions:
            lidar_folder = os.path.join(input_path, 'point_clouds', dir)
            self.lidar_list += (get_files(lidar_folder, 'pcd'))

    def create_yolo_obb_dataset(self, test_fraction=0.2, val_fraction=0.2):
        assert self.output_path is not None, "Output folder must be not be None"

        val_size, test_size = int(
            len(self.lidar_list)*val_fraction), int(len(self.lidar_list)*test_fraction)
        train_size = len(self.lidar_list) - val_size - test_size
        assert train_size > 0, "Invalid train/val/test split."

        shuffle_list(self.lidar_list)

        folders = [('train', range(0, train_size)), ('val', range(
            train_size, val_size+train_size)), ('test', range(train_size+val_size, test_size+val_size+train_size))]
        for folder, split_range in folders:
            img_path = os.path.join(self.output_path, 'images', folder)
            gt_path = os.path.join(self.output_path, 'labels', folder)
            os.makedirs(img_path)
            os.makedirs(gt_path)

            for idx in split_range:
                bev_image, det_list = self.get_bev_and_label(idx=idx)
                det_list = self.normalize_labels(det_list)

                file_name = str(idx).zfill(7)
                self.write_label_file(
                    det_list, name=gt_path + f'/{file_name}.txt')
                cv2.imwrite(
                    img_path + f'/{file_name}.jpg', array_to_image(bev_image))

                print(f'Saving image/label {idx} to {folder}')

    @staticmethod
    def normalize_labels(labels):
        labels = np.array(labels).astype(float)
        labels[:, [1, 3, 5, 7]] = labels[:, [1, 3, 5, 7]] / cfg.BEV_WIDTH
        labels[:, [2, 4, 6, 8]] = labels[:, [2, 4, 6, 8]] / cfg.BEV_HEIGHT

        # Clip labels
        labels[:, [1, 3, 5, 7]] = np.clip(labels[:, [1, 3, 5, 7]], 0, 1)
        labels[:, [2, 4, 6, 8]] = np.clip(labels[:, [2, 4, 6, 8]], 0, 1)
        return labels.tolist()

    @staticmethod
    def write_label_file(data, name):
        with open(name, 'w') as file:
            for d in data:
                entry = f'{int(d[0])} {d[1]} {d[2]} {d[3]} {d[4]} {d[5]} {d[6]} {d[7]} {d[8]} \n'
                file.write(entry)

    def demo_pc_to_image(self, debug=False):
        for idx in range(len(self.lidar_list)):
            self.get_bev_and_label(idx=idx, visualize=True, debug=debug)

    def get_bev_and_label(self, idx: int, visualize=False, debug=False):
        pc_path = self.lidar_list[idx]
        pc = self.get_pc(pc_path)

        # Normalize pointcloud orientation and height, align road plane with x-y plane
        '''
        TODO add transform that rotates yaw angle for better cropping
        OR use a transformed cropbox that is the size of the RoI 
        '''
        pc = transform_pc(pc, cfg.lidar2ground)

        # Crop point cloud based on paramters
        pc = pc[np.logical_not((pc[:, 0] <= cfg.boundary['minX']) | (
            pc[:, 0] > cfg.boundary['maxX']))]
        pc = pc[np.logical_not((pc[:, 1] <= cfg.boundary['minY']) | (
            pc[:, 1] > cfg.boundary['maxY']))]
        pc = pc[np.logical_not((pc[:, 2] <= cfg.boundary['minZ']) | (
            pc[:, 2] > cfg.boundary['maxZ']))]

        # Apply radius removal
        pc = radius_outlier_removal(pc, num_points=12, r=0.8)

        # Get detection bboxes in the ground plane
        gt_json = get_gt(pc_path)
        det_list = self.convert_a9_json(gt_json)
        det_list = self.crop_labels(
            det_list, height=cfg.BEV_HEIGHT, width=cfg.BEV_WIDTH)

        # Convert to BEV
        bev_image = self.create_bev(pc, visualize=visualize, labels=det_list)

        if debug:
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(pc)
            triad = o3d.geometry.TriangleMesh.create_coordinate_frame(
                size=10, origin=[0, 0, 0])
            o3d.visualization.draw_geometries([pcd, triad])

        return bev_image, det_list

    @staticmethod
    def crop_labels(labels, height, width):
        labels = np.array(labels).astype(float)
        # All 4 points must be outside image to be deleted
        mask = ~((labels[:, 1] > width) | (labels[:, 2] > height) &
                 (labels[:, 3] > width) | (labels[:, 4] > height) &
                 (labels[:, 5] > width) | (labels[:, 6] > height) &
                 (labels[:, 7] > width) | (labels[:, 8] > height))

        return labels[mask].tolist()

    def create_bev(self, pc, visualize=False, labels=None):
        '''
        Based on: https://github.com/maudzung/SFA3D
        
        create 3 channel image
        1) density
        2) height map
        3) Options: range image (dist2sensor), surface normals?
        '''
        Height = cfg.BEV_HEIGHT + 1
        Width = cfg.BEV_WIDTH + 1

        # Discretize Feature Map
        PointCloud = np.copy(pc)

        range = np.sqrt(pow(PointCloud[:, 0], 2.0) +
                        pow(PointCloud[:, 1], 2.0)).reshape(-1, 1)
        PointCloud = np.hstack([PointCloud, range])

        PointCloud[:, 0] = np.int_(
            np.floor(PointCloud[:, 0] / cfg.DISCRETIZATION))
        PointCloud[:, 1] = np.int_(
            np.floor(PointCloud[:, 1] / cfg.DISCRETIZATION) + Width / 2)

        # sort-3times
        sorted_indices = np.lexsort(
            (-PointCloud[:, 2], PointCloud[:, 1], PointCloud[:, 0]))
        PointCloud = PointCloud[sorted_indices]
        _, unique_indices, unique_counts = np.unique(
            PointCloud[:, 0:2], axis=0, return_index=True, return_counts=True)
        PointCloud_top = PointCloud[unique_indices]

        # Height Map, Intensity Map & Density Map
        heightMap = np.zeros((Height, Width))
        rangeMap = np.zeros((Height, Width))
        densityMap = np.zeros((Height, Width))

        max_height = float(np.abs(cfg.boundary['maxZ'] - cfg.boundary['minZ']))
        heightMap[np.int_(PointCloud_top[:, 0]), np.int_(
            PointCloud_top[:, 1])] = PointCloud_top[:, 2] / max_height

        max_range = PointCloud[:, 3].max()
        rangeMap[np.int_(PointCloud_top[:, 0]), np.int_(
            PointCloud_top[:, 1])] = PointCloud_top[:, 3] / max_range

        normalizedCounts = np.minimum(
            1.0, np.log(unique_counts + 1) / np.log(64))
        densityMap[np.int_(PointCloud_top[:, 0]), np.int_(
            PointCloud_top[:, 1])] = normalizedCounts

        RGB_Map = np.zeros((3, Height - 1, Width - 1))
        RGB_Map[2, :, :] = densityMap[:cfg.BEV_HEIGHT, :cfg.BEV_WIDTH]  # r_map
        RGB_Map[1, :, :] = heightMap[:cfg.BEV_HEIGHT, :cfg.BEV_WIDTH]  # g_map
        RGB_Map[0, :, :] = rangeMap[:cfg.BEV_HEIGHT, :cfg.BEV_WIDTH]  # b_map

        if visualize:
            image = array_to_image(RGB_Map)
            if labels is not None:
                image = self.annotate_bev(labels, image)

            cv2.imshow('Numpy Array as Image', image)
            if self.user_input_handler() < 0:
                exit(0)

        return RGB_Map

    def convert_a9_json(self, gt_json):
        '''
        Covnert the A9 .json gt format into lists of yolo-obb format
        list = [[class, x1, y1, x2, y2, x3, y3, x4, y4]]
        '''
        frames = gt_json['openlabel']['frames']

        det_list = []

        # Only one frame per file
        frame_num = list(frames.keys())[0]
        objects = frames[frame_num]['objects']
        for item in objects.items():
            data = item[1]['object_data']
            bbox = data['cuboid']['val']
            bbox_bev = self.convert_labels(bbox)
            corners = self.bbox3d_to_corners(bbox_bev)
            detection = [cfg.CLASS_NAME_TO_ID[data['type']]] + corners[:]
            det_list.append(detection)
        return det_list

    @staticmethod
    def user_input_handler() -> int:
        out = 0
        key = cv2.waitKey(0)

        # Escape key
        if (key == 27):
            out = -1
        return out

    @staticmethod
    def bbox3d_to_corners(bbox_bev):
        '''
        Take a9 label cuboid format to corners
        '''
        x, y, z = bbox_bev[0], bbox_bev[1], bbox_bev[2]
        w, l, h = bbox_bev[3], bbox_bev[4], bbox_bev[5]
        yaw = bbox_bev[6]

        sin_yaw = np.sin(yaw)
        cos_yaw = np.cos(yaw)

        # Rotate the point and then add absolute position
        x1 = (l/2 * cos_yaw - w/2 * sin_yaw) + x
        y1 = (l/2 * sin_yaw + w/2 * cos_yaw) + y

        x2 = (-l/2 * cos_yaw - w/2 * sin_yaw) + x
        y2 = (-l/2 * sin_yaw + w/2 * cos_yaw) + y

        x3 = (-l/2 * cos_yaw + w/2 * sin_yaw) + x
        y3 = (-l/2 * sin_yaw - w/2 * cos_yaw) + y

        x4 = (l/2 * cos_yaw + w/2 * sin_yaw) + x
        y4 = (l/2 * sin_yaw - w/2 * cos_yaw) + y

        return [x1, y1, x2, y2, x3, y3, x4, y4]

    def convert_labels(self, bbox):
        '''
        Convert A9 label data into the psuedo image pixel space
        '''
        x, y, z = bbox[0], bbox[1], bbox[2]
        qx, qy, qz, qw = bbox[3], bbox[4], bbox[5], bbox[6]
        w, l, h = bbox[7], bbox[8], bbox[9]

        yaw = euler_from_quaternion(qw, qx, qy, qz)

        yaw = -yaw
        y1 = int((x - cfg.boundary['minX']) / cfg.DISCRETIZATION)
        x1 = int((y - cfg.boundary['minY']) / cfg.DISCRETIZATION)
        z1 = z
        w1 = int(w / cfg.DISCRETIZATION)
        l1 = int(l / cfg.DISCRETIZATION)
        h1 = h

        return x1, y1, z1, w1, l1, h1, yaw

    def annotate_bev(self, labels, image):
        for obj in labels:
            colour = cfg.colours[int(obj[0])]
            image = self.draw_r_bbox(obj[1:], image, colour)
        return image

    def draw_r_bbox(self, corners, img, colour):
        '''
        Draw rotated bbox on the psuedo image
        '''
        corners_int = np.array(corners).astype(int)

        img = cv2.line(img, (corners_int[0], corners_int[1]),
                       (corners_int[2], corners_int[3]), colour, 2)
        img = cv2.line(img, (corners_int[2], corners_int[3]),
                       (corners_int[4], corners_int[5]), colour, 2)
        img = cv2.line(img, (corners_int[4], corners_int[5]),
                       (corners_int[6], corners_int[7]), colour, 2)
        img = cv2.line(img, (corners_int[6], corners_int[7]),
                       (corners_int[0], corners_int[1]), colour, 2)

        return img

    def get_pc(self, lidar_file):
        pc = PointCloud.from_path(lidar_file)
        return pc.numpy()


def main(args):
    lbc = A9LidarBevCreator(input_path=args.input, output_path=args.output)

    # Process the point clouds into images
    # lbc.demo_pc_to_image(debug=False)

    # Save a sample image
    # array, gt = lbc.get_bev_and_label(0)
    # img = array_to_image(array)
    # print(gt)
    # print(len(gt))
    # cv2.imwrite('./test_a9_lidar_img.jpg', img)

    lbc.create_yolo_obb_dataset()


# check if the script is run directly and call the main function
if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Processes the LiDAR images into BEV psuedo images in the YOLO-obb format.")
    parser.add_argument(
        "-i", "--input", help="The path of the A9 sequence.", type=str,
        default='/home/adrian/dev/A9_images_and_points/a9_dataset_r02_s01')
    parser.add_argument(
        "-o", "--output", help="The path where the results are saved.", default='/home/adrian/dev/A9_images_and_points/bev_lidar1')
    args = parser.parse_args()
    main(args)
