import numpy as np
import open3d as o3d

# read a pcd file
pcd = o3d.io.read_point_cloud(
    "/home/adrian/dev/metrics/dec7_daytime_2/000240.pcd")
# convert to numpy array
pc_array = np.asarray(pcd.points, dtype=np.float32)
pc_array = np.hstack((pc_array, np.zeros((pc_array.shape[0], 1))))

# save as npy file
np.save("/home/adrian/dev/metrics/test_cloud_marc_roof.npy", pc_array)


"""
If we want to write a file with intesity
"""
# # load a npy file
# pc_array = np.load("your_file.npy")
# # convert to o3d point cloud
# pcd = o3d.geometry.PointCloud()
# pcd.points = o3d.utility.Vector3dVector(pc_array[:, :3])
# # set the intensity channel as colors
# pcd.colors = o3d.utility.Vector3dVector(pc_array[:, 3:])
# # save as pcd file
# o3d.io.write_point_cloud("your_file.pcd", pcd)
