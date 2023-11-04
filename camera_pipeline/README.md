# camera_pipeline

## Directory Structure
```
├── camera_pipeline
│   ├── config
│   │   └── rviz
│   ├── camera_pipeline
│   │   ├── camera_processing
│   │   │   ├── trackers
│   │   │   ├── weights
│   │   │   └── yolov5
│   │   └── lidar_processing
│   │       └── utils
│   ├── launch
│   ├── resource
│   └── test
```
<!---
tree -d -L 3 -I __pycache__
--->

> **Note:** This is setup like a ROS2 package, but you do not need to build it to run the scripts. However, building is requried to use the launch files.

# Setup

<!-- To get started
```
git clone --recurse-submodules https://github.com/adrian-soch/camera_pipeline.git
``` -->

<!-- ### ROS2 Packages

#### ros2_numpy
```
cd <ROS2_WORKSPACE>/src
git clone https://github.com/Box-Robotics/ros2_numpy.git
cd <ROS2_WS>
colcon build
. install/setup.bash
``` -->

### Install Python requirements
```
# PyTorch CPU Version
pip install torch torchvision

# GPU Version go here: https://pytorch.org/get-started/locally/

pip install numpy

# OpenCV CPU Version
pip install opencv-python
pip install opencv-contrib-python
```

### Yolov5 requirements
```
numpy>=1.18.5
opencv-python>=4.1.1
Pillow>=7.1.2
psutil  # system resources
PyYAML>=5.3.1
requests>=2.23.0
scipy>=1.4.1
```

If I missed any requirements that cause an error at runtime, just install any packages that are missing.

<!-- ## OPTIONAL
```
cd <ROS2_WORKSPACE>/src
git clone --recurse-submodules https://github.com/adrian-soch/camera_pipeline.git
cd <ROS2_WS>
colcon build
```
# Development notes

After updating files you must 
```
cd <ROS2 WORSKAPCE>
colcon build
. install/setup.bash
``` -->

<!-- #### Optional
To clean the ros2 workspace run `rm -rf log/ install/ build/`. Warning `rm -rf` means deleting a folder without the ability to recover it.

> Use git and commit often. -->

# Visualizing Data

Before you start you need to source ros2 in all the terminal you want to run a command in.

1. Run `ros2 bag play <PATH TO BAG FOLDER>`
1. In another terminal run `rviz2`
1. At the bottom of the Display sidebar click Add. In the topic tab, find pointcloud. Note: you may need to change point size and zoom out to see the data. Repeat for adding the image.
   - If you cant see the point cloud data.
     - Select a different `fixed frame` in the `Global Options`.
     - Change topic->Reliability to Best effort.
     - For data without a transform, in another terminal:
  ```ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 world map```. Here `map` is found from the frame_id from the command `ros2 topic echo /<Topic name>`.
  - 

# Running the Code

#### Image detection and tracking:
```
# Play the rosbag
ros2 bag play <Path to rosbag>/<bag name>.db3

# Start the python code
cd <Path to camera_pipeline>/camera_pipeline
python3 camera_processing_node.py
```
To modify the parameters go to `camera_pipeline/camera_processing/vision_track.py`.


#### Starting the LiDAR
```
# Ensure your source ROS2 and run `. install/setup.bash`
ros2 launch camera_pipeline ouster_driver_launch.py
``'

## References

1. https://github.com/ultralytics/yolov5
2. https://github.com/mikel-brostrom/Yolov5_StrongSORT_OSNet
3. https://github.com/ros-drivers/ros2_ouster_drivers
4. https://static.ouster.dev/sensor-docs/index.html
