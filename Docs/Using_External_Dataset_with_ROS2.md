# How to use external datsets with the pipeline

## Part 1: How to convert A9 dataset into ROS bags

1. Update the folder paths in `file2bag.launch.py`. Build the package so the updated launch file is copied to the execution folder (not necessary if you have previous built with symlinks on).
2. Execute `ros2 launch data_tools file2bag.launch.py`
3. Wait until all the files are parsed
4. Play the ros bag and manually check the replay is correct in `Rviz2`

> **Note:** this script assumes there are an equal number of image and pointcloud files. AND it assumes the files are in order and syncronized.

## Part 2: Find the Sensor2Ground Transformation and Crop Box parameters

1. Execute `ros2 launch cam_lidar_bringup transform_explorer.launch.py`
2. In a seperate terminal run `ros2 run data_tools roi_visualizer_node.py`.
3. In another terminal play your ros bag, optionally adding `-l` to loop the bag if it is short in duration.
4. Change the parameters for translation and orientation in `RQT` until the ground plane is level with the grid in `Rviz2`.
5. Change the paramters for the crop box until the cube contains the region you are interested in.
6. Record the results and manually enter them into a configuration file just like `lidar_pipeline/configs/dec7_config.yaml`.

> **Note:** results are printed to the terminal.