# cam_lidar_tools

```
├── cam2image
│   ├── include
│   │   └── cam2image
│   ├── launch
│   └── src
├── data_tools
│   ├── include
│   │   └── data_tools
│   ├── scripts
│   └── src
├── Docs
├── fusion_engine
│   ├── config
│   │   └── rviz
│   ├── fusion_engine
│   │   ├── camera_processing
│   │   └── lidar_processing
│   ├── launch
│   ├── resource
│   └── test
├── lidar_pipeline
│   ├── configs
│   ├── include
│   │   └── lidar_pipeline
│   ├── launch
│   └── src
├── Metrics
│   └── kitti_MOT_2D_HOTA_Test
│       └── data
├── obj_tracker
│   ├── obj_tracker
│   │   └── sort
│   ├── resource
│   └── test
└── ros2_numpy
    ├── ros2_numpy
    └── test

```
<!---
tree -d -L 3 -I __pycache__
--->

## RQT Graph

![image](https://github.com/adrian-soch/cam_lidar_tools/assets/6884645/fcae153a-d175-48d1-a33a-fc8e5faa0b82)

![image](https://github.com/adrian-soch/cam_lidar_tools/assets/6884645/1fcd223e-6cf9-4caf-a0e6-907f08eb147d)

---
# Description

## ROS Packges
> `fusion_engine`: Camera object detection and tracking module

> `cam2image`: USB Camera driver

> `ros2_numpy`: Fork of repo with tools for converting msgs to numpy

> `lidar_pipeline`: LiDAR Object Detection Module

> `obj_tracker`: LiDAR Object Tracking Module

> `data_tools`: Tools for converting and processing data

## Non-ROS packages

> Docs: Files and info that should be retained but not in the code

> Metrics: Scripts and info about calculating performance metrics

---
# Install

## Install Python requirements
```
# PyTorch CPU Version
pip install torch torchvision

# GPU Version
See: https://pytorch.org/get-started/locally/

pip install numpy

# OpenCV CPU Version
pip install opencv-python
pip install opencv-contrib-python
```

### Yolov5/7 requirements
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

## C++ requirements

```
sudo apt install libtins-dev
```
## ROS2 requirements
Install via `apt`:
```
ros-galactic-pcl-ros
ros-galactic-ament-cmake-nose
```

## Clone and build repo

```
cd <ROS2_WS>/src
git clone --recurse-submodules https://github.com/adrian-soch/cam_lidar_tools.git
cd ..
colcon build
```
> Always source the ROS2 install and the local ros2 wroksapce via `source /opt/ros/galactic/setup.bash` and `. install/setup.bash` respectively.

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

