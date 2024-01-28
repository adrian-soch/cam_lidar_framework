import math
import numpy as np

# Emergency_vehicle and van merged
CLASS_NAME_TO_ID = {'BICYCLE':0, 'BUS':1, 'CAR':2, 'EMERGENCY_VEHICLE':3,
                   'MOTORCYCLE':4, 'PEDESTRIAN':5, 'TRAILER':6, 'TRUCK':7, 'VAN':3}

colors = [[0, 255, 255], [0, 0, 255], [255, 0, 0], [255, 120, 0],
          [255, 120, 120], [0, 120, 0], [120, 255, 255], [120, 0, 255]]

#####################################################################################
boundary = {
    "minX": 0,
    "maxX": 50,
    "minY": -25,
    "maxY": 25,
    "minZ": -0.2,
    "maxZ": 3.8,
}

bound_size_x = boundary['maxX'] - boundary['minX']
bound_size_y = boundary['maxY'] - boundary['minY']
bound_size_z = boundary['maxZ'] - boundary['minZ']

BEV_WIDTH = 1024
BEV_HEIGHT = 1024

# DISCRETIZATION = (boundary["maxX"] - boundary["minX"]) / BEV_HEIGHT

# # maximum number of points per voxel
# T = 35

# # voxel size
# vd = 0.1  # z
# vh = 0.05  # y
# vw = 0.05  # x

# # voxel grid
# W = math.ceil(bound_size_x / vw)
# H = math.ceil(bound_size_y / vh)
# D = math.ceil(bound_size_z / vd)

# Parameters for all North LiDAR in A9 dataset
#####################################################################################
Tr_velo_to_cam = np.array([
    [7.49916597e-03, -9.99971248e-01, -8.65110297e-04, -6.71807577e-03],
    [1.18652889e-02, 9.54520517e-04, -9.99910318e-01, -7.33152811e-02],
    [9.99882833e-01, 7.49141178e-03, 1.18719929e-02, -2.78557062e-01],
    [0, 0, 0, 1]
])

# cal mean from train set
R0 = np.array([
    [0.99992475, 0.00975976, -0.00734152, 0],
    [-0.0097913, 0.99994262, -0.00430371, 0],
    [0.00729911, 0.0043753, 0.99996319, 0],
    [0, 0, 0, 1]
])

P2 = np.array([[719.787081, 0., 608.463003, 44.9538775],
               [0., 719.787081, 174.545111, 0.1066855],
               [0., 0., 1., 3.0106472e-03],
               [0., 0., 0., 0]
               ])

R0_inv = np.linalg.inv(R0)
Tr_velo_to_cam_inv = np.linalg.inv(Tr_velo_to_cam)
P2_inv = np.linalg.pinv(P2)
#####################################################################################
