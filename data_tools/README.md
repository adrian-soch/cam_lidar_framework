# Data Tools

## Converting .pcd and Images into ROS Bags

Use `file2bag.launch.py` to read files from a folder and save to a ros bag. To use, just update the folder paths in the launch file, perform a `colcon_build` then start the launch file.

**Note:** the script is slow and cannot publish at 10Hz. So intead we publish at 2.5 hz, then when replaying the bag play it back 4x faster (2.5 * 4 = 10hz). To do this use add `-r 4` to the `ros2 bag play` command.

## Converting ROS bags into seperate file

Use `synced_image_cloud_2file.cpp` for saving pointclouds and images to seperate files.

## Region of Interest (RoI) Vizualization Tool

To quickly rifgure out the parameters of a cuboid object in the 3D pointcloud space, run `roi_visualizer_node.py`. Then use `rqt_gui` and the `Parameter Reconfigure` tab to make change the poaranters and see live updates in Rviz to the cuboid object.

This is mainly used for determining the paramters to the `crop box` object in the trad_lidar_detector.
