import numpy as np
import open3d as o3d

pc_array = np.load("/home/adrian/dev/metrics/test_cloud_marc_roof.npy")
# convert to o3d point cloud
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(pc_array[:, :3])
# set the intensity channel as colors
# pcd.colors = o3d.utility.Vector3dVector(pc_array[:, 3:])
# display the pointcloud on the screen
o3d.visualization.draw_geometries([pcd])
