# cam_lidar_tools

This repositiory contains ROS 2 packages for realtime LiDAR + Camera fusion for road-side traffic monitoring applications.

![image](./Docs/readme_images/fusion_demo.png)

```
├── cam2image
│   ├── include
│   ├── launch
│   └── src
├── camera_pipeline
│   ├── camera_pipeline
│   ├── config
│   ├── launch
│   ├── resource
│   └── test
├── data_tools
│   ├── data_tools
│   ├── launch
│   ├── scripts
│   └── src
├── Docs
│   └── readme_images
├── fusion_module
│   ├── fusion_module
│   ├── resource
│   └── test
├── lidar_pipeline
│   ├── configs
│   ├── include
│   ├── launch
│   └── src
├── obj_classifier
│   ├── obj_classifier
│   ├── resource
│   └── test
├── obj_tracker
│   ├── obj_tracker
│   ├── resource
│   └── test
├── pipeline_interfaces
│   ├── msg
│   └── srv
└── ros2_numpy
    ├── ros2_numpy
    └── test
```
<!---
tree -d -L 2 -I __pycache__
--->

## RQT Graph

![image](./Docs/readme_images/rosgraph.png)

---
# Description

## ROS Packges
> `camera_pipeline`: Camera object detection and tracking module

> `cam2image`: USB Camera driver

> `ros2_numpy`: Fork of repo with tools for converting msgs to numpy

> `lidar_pipeline`: LiDAR Object Detection Module

> > `obj_classifier`: LiDAR Object Classification Module

> `obj_tracker`: LiDAR Object Tracking Module

> `data_tools`: Tools for converting and processing data

> `fusion_module`: LiDAR + Camera Fusion module 

## Non-ROS packages

> Docs: Files and info that should be retained but not in the code

> Metrics: Scripts and info about calculating performance metrics

---
# Setup

Tested on:

| Ubunbtu 20.04 |
|:-------------:|
|  i7 16 cores  |
|   32 GB RAM   |
|  Discrete GPU |

## 1. ROS 2 Galactic 
Install ROS 2 Galactic.

## 2. Install Python requirements
```
# PyTorch
# GPU Version
# See: https://pytorch.org/get-started/locally/

pip install numpy

# OpenCV CPU Version
pip install opencv-python
pip install opencv-contrib-python
```

### 2.1 Yolov5/7 requirements
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

## 3. C++ requirements

```
sudo apt install libtins-dev
```
## 4. ROS2 requirements
Install via `apt`:
```
ros-galactic-pcl-ros
ros-galactic-ament-cmake-nose
```

## 5. Clone and build repo

```
cd <ROS2_WS>/src
git clone --recurse-submodules https://github.com/adrian-soch/cam_lidar_tools.git
cd ..
colcon build
```
> Always source the ROS2 install and the local ros2 worksapce via `source /opt/ros/galactic/setup.bash` and `. install/setup.bash` respectively.

---
# Running the code

1. Download the ROS bags from: `TBD`.

2. Run the launch file in the terminal `ros2 launch lidar_pipeline fusion_demo.launch.py`. The primary launch file will start all the nodes including Rviz2 for visualization. See: `cam_lidar_tools/lidar_pipeline/launch/fusion_demo.launch.py`, there are some parameters you can adjust there for different demonstrations.

3. Tune parameters and re-run. inspect the launch file to see what nodes are being executed. Sometime the launch file calls other launch files. Each node may have different paramters that can be adjusted.

---
# Troubleshooting and Comments

## Colcon build fails?
- Check that the libraries you import are in your `CMakeLists.txt` (C++) or `setup.py` (Python) files.
- Certain warning disappera if you build a 2nd time.

## Known issues

- cmake warning for PCL lib. Just re-build with the same command you used and the warning will be gone

## Comments

- Use git and commit often.
- To clean the ros2 workspace run `rm -rf log/ install/ build/`. Warning `rm -rf` means deleting a folder without the ability to recover it.

