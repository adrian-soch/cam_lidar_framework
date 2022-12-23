#! /usr/bin/env python3
"""
@file projection.py

@brief This iterates through a rosbag and projects pc2 onto an image.
    Timestamps are not checked, images and pcs'2 adjacent to each other areassumed to be
    syncronized.

@section Author(s)
- Created by Adrian Sochaniwsky on 3/12/2022
"""

import cv2
import time
from matplotlib import pyplot as plt
from numpy.lib.recfunctions import structured_to_unstructured

from sensor_msgs.msg import PointCloud2, PointField
import ros2_numpy

from cv_bridge import CvBridge
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr

from utils.projection_utils import *

def main():
    CAM_CAL_PATH = '/home/adrian/dev/ros2_ws/src/fusion_engine/config/1280x720_ost.yaml'
    TRANSFORM_PATH = '/home/adrian/dev/ros2_ws/src/fusion_engine/config/transforms.yaml'
    ROSBAG_PATH = '/home/adrian/dev/bags/dec12_2022/HD_lidar_ROS_RECEP_0'

    pc2_topic ='/points'
    image_topic = '/image'

    camera_mat, dist = get_calib_from_file(CAM_CAL_PATH, ['camera_matrix', 'distortion_coefficients'], intrinsic=True)
    transforms = ['lidar2cam_extrinsic', 'lidarData2lidarSensor_extrinsic']
    [l2c_mat, ld2ls_mat] = get_calib_from_file(TRANSFORM_PATH, transforms)
    
    # Must be 4x4
    ld2ls_mat = np.vstack([ld2ls_mat, np.array([0,0,0,1], dtype=np.float32)])

    cmap = plt.cm.get_cmap('hsv', 256)
    cmap = np.array([cmap(i) for i in range(256)])[:, :3] * 255
    
    br = CvBridge()

    # Read recording
    lidar_ts, cam_ts = 0, 0
    img, pc = None, None
    with Reader(ROSBAG_PATH) as reader:

        # topic and msgtype information is available on .connections list
        for connection in reader.connections:
            print(connection.topic, connection.msgtype)

        # iterate over messages
        have_image = False
        have_pc = False
        for connection, timestamp, rawdata in reader.messages():
            msg = deserialize_cdr(rawdata, connection.msgtype)

            start_s = time.time()

            # if connection.topic != '/tf_static':
            #     print(msg.header.stamp.nanosec, connection.msgtype)

            if connection.topic == pc2_topic:
                msg = deserialize_cdr(rawdata, connection.msgtype)
                lidar_ts = msg.header.stamp.nanosec


                f1 = PointField(name='x',offset=0,datatype=7,count=1)
                f2 = PointField(name='y',offset=4,datatype=7,count=1)
                f3 = PointField(name='z',offset=8,datatype=7,count=1)
                f4 = PointField(name='intensity',offset=16,datatype=7,count=1)
                f5 = PointField(name='t',offset=20,datatype=6,count=1)
                f6 = PointField(name='reflectivity',offset=24,datatype=4,count=1)
                f7 = PointField(name='ring',offset=26,datatype=2,count=1)
                f8 = PointField(name='ambient',offset=28,datatype=4,count=1)
                f9 = PointField(name='range',offset=32,datatype=6,count=1)

                temp = PointCloud2(data=msg.data, \
                    height = msg.height,\
                    width = msg.width,\
                    is_dense = msg.is_dense,\
                    point_step = msg.point_step,\
                    row_step = msg.row_step,\
                    fields=[f1,f2,f3,f4,f5,f6,f7,f8,f9])

                pc = ros2_numpy.numpify(temp)

                # pc = pc2_numpy(msg)
                have_pc = True
                
            elif connection.topic == image_topic:
                msg = deserialize_cdr(rawdata, connection.msgtype)
                img = br.imgmsg_to_cv2(msg)
                img = cv2.flip(img, -1)
                have_image = True
            else:
                msg = deserialize_cdr(rawdata, connection.msgtype)
            
            # Reader reads 1 message at a time
            #   We need to go through each msg before we have
            #   A complete set of msgs
            if not(have_image and have_pc):
                continue
            else:
                have_image = False
                have_pc = False

            '''
            Transform pc
            ###################################################
            '''

            # Only retain x, y, z, intensity
            pc = pc.reshape(pc.size)
            pc = pc[['x','y','z', 'intensity']]

            # Convert to unstructured so we can operate easier
            pc = structured_to_unstructured(pc)

            # Remove points behind lidar (in lidar frame this is x-axis)
            # This also removes (0,0,0) points, because they start with x=-0.0
            pc = pc[np.logical_not(pc[:,0] <= 0)]

            # Return x,y,z,1
            xyz1 = np.hstack([pc[:, :3], np.ones((pc.shape[0], 1), dtype=np.float32)])

            # from dataFrame -> sensorFrame -> cameraFrame
            #       (3*4) @ (4*4) @ (4*N) = (3*N)
            temp = l2c_mat @  ld2ls_mat @ xyz1.T
            
            '''
            Project pc to img
            ###################################################
            '''
            # Get projection on image plane
            # (3*3) @ (3*N) = (3*N)
            px_proj = (camera_mat @ temp)

            # Get depth
            depth = px_proj[2 :].T

            # Divide by depth to get 2-D projection
            px_proj = (px_proj[:2,:] / px_proj[2 :]).T

            # Restrict points to camera fov
            depth = depth[np.logical_not(np.logical_or(px_proj[:,0] < 0, px_proj[:,0] > img.shape[1]))]
            px_proj = px_proj[np.logical_not(np.logical_or(px_proj[:,0] < 0, px_proj[:,0] > img.shape[1]))]

            depth = depth[np.logical_not(np.logical_or(px_proj[:,1] < 0, px_proj[:,1] > img.shape[0]))]
            px_proj = px_proj[np.logical_not(np.logical_or(px_proj[:,1] < 0, px_proj[:,1] > img.shape[0]))]  


            norm = plt.Normalize()
            colors = np.int32(plt.cm.rainbow(norm(depth),bytes=True))
            colors = colors[:,0,0:3]
      
            for idx,i in enumerate(px_proj):
                c = tuple(map(int, colors[idx]))
                cv2.circle(img, np.int32(i), 2, c, -1)

            cv2.imshow('proj', img)
            cv2.waitKey(1)

            # print('Time: ' + str(time.time() - start_s))
            print(f'Time {(time.time() - start_s)*1000.0:3.6}')
            

if __name__ == '__main__':
    main()