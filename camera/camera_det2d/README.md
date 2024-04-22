# camera_det2d

## Directory Structure
```
├── camera_det2d
├── config
│   └── rviz
├── launch
├── resource
├── scripts
├── test
└── weights

```
<!---
tree -d -L 3 -I __pycache__
--->

# Setup


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
pip install ultralytics
pip install numpy
pip install opencv-python
pip install opencv-contrib-python
```

If I missed any requirements that cause an error at runtime, just install any packages that are missing.

## OPTIONAL

# Development notes

After updating files you must
```
cd <ROS2 WORSKAPCE>
colcon build
. install/setup.bash
```

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
ros2 bag play <Path to rosbag folder>

# Start the node
ros2 launch camera_det2d cam_processor_launch.py
```
To modify model inference parameters, you can directly modify the model prediction call.

#### Starting the LiDAR
```
# Ensure your source ROS2 and run `. install/setup.bash`
ros2 launch camera_det2d ouster_driver_launch.py
``'

## References

1. https://github.com/ultralytics/yolov5
2. https://github.com/mikel-brostrom/Yolov5_StrongSORT_OSNet
3. https://github.com/ros-drivers/ros2_ouster_drivers
4. https://static.ouster.dev/sensor-docs/index.html
