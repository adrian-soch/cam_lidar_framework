# cam_lidar_tools

```
├── cam2image
│   ├── include
│   │   └── cam2image
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
├── ros2_numpy
│   ├── ros2_numpy
│   └── test
└── ros2_ouster_drivers
    ├── ouster_msgs
    │   ├── msg
    │   └── srv
    └── ros2_ouster
        ├── design
        ├── include
        ├── launch
        ├── params
        └── src

```
<!---
tree -d -L 3 -I __pycache__
--->

# Description
> `fusion_engine`: Camera + LiDAR object detection and tracking module

> `cam2image`: Camera driver

> `ros2_numpy`: Fork of repo with tools for converting msgs to numpy

# Install

## Install Python requirements
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

## C++ requirements

```
sudo apt install libtins-dev
```
## Clone and build repo

```
cd <ROS2_WS>/src
git clone --recurse-submodules https://github.com/adrian-soch/cam_lidar_tools.git
cd ..
colcon build
```
After building always source the ROS2 install and the local ros2 wroksapce via `source /opt/ros/galactic/setup.bash` and `. install/setup.bash` respectively.

## Troubleshooting and Comments

### Known issues

- cmake warning for PCL lib. Just re-build with the same command you used and the warning will be gone

### Comments

- Use git and commit often.
- To clean the ros2 workspace run `rm -rf log/ install/ build/`. Warning `rm -rf` means deleting a folder without the ability to recover it.

