import cv2
import json
from glob import glob
import os
import open3d as o3d
import numpy as np
import random
random.seed(69)

def shuffle_list(list:list) -> None:
    random.shuffle(list)

def get_files(path, ext:str) -> list:
    assert os.path.isdir(path)
    files = glob(os.path.join(path, f'*.{ext}'))
    return sorted(files)

def save_img(filename:str, cv_image):
    cv2.imwrite(filename, cv_image)

def array_to_image(array) -> np.ndarray:
        image = (array*255).astype(np.uint8)
        image = image.transpose((1, 2, 0))  # HWC to CHW
        image = np.ascontiguousarray(image, dtype=np.uint8)
        return image

def create_black_img(height:int, width:int) -> np.ndarray:
    return np.zeros((3, height, width))

def transform_pc(pc, transform) -> np.ndarray:
    # Return x,y,z,1
    xyz1 = np.hstack(
        [pc[:, :3], np.ones((pc.shape[0], 1), dtype=np.float32)])
    xyz1 = np.matmul(transform, xyz1.T).T

    pc[:, :3] = xyz1[:, :3]
    return pc

def get_gt(lidar_file) -> dict:
    head, tail = os.path.split(lidar_file)
    name, _ = os.path.splitext(tail)
    gt_path = os.path.join(head.replace("/point_clouds/", "/labels_point_clouds/"),
                           name + '.json')

    with open(gt_path) as f:
        return json.load(f)

def radius_outlier_removal(pc, num_points=12, r=0.8) -> np.ndarray:
    pc = pc.T if pc.shape[1] > 9 else pc
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pc[:, :3])
    _, ind = pcd.remove_radius_outlier(nb_points=num_points, radius=r)

    mask = np.zeros(pc.shape[0], dtype=bool)
    mask[ind] = True
    return pc[mask]

def euler_from_quaternion(qw, qx, qy, qz) -> float:
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)

    Note: only returns yaw about z axis
    """
    # t0 = +2.0 * (q.w * q.x + q.y * q.z)
    # t1 = +1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    # roll_x = np.atan2(t0, t1)

    # t2 = +2.0 * (q.w * q.y - q.z * q.x)
    # t2 = +1.0 if t2 > +1.0 else t2
    # t2 = -1.0 if t2 < -1.0 else t2
    # pitch_y = np.asin(t2)

    t3 = +2.0 * (qw * qz + qx * qy)
    t4 = +1.0 - 2.0 * (qy * qy + qz * qz)
    yaw_z = np.arctan2(t3, t4)

    return yaw_z  # in radians