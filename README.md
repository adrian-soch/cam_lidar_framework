# cam_lidar_tools

This repositiory contains ROS 2 packages for realtime Camera-LiDAR Fusion for static roadside traffic monitoring applications.

<p align="center">
        <img src="./Docs/readme_images/fusion_demo.png" alt="drawing" width="1000"/>
</p>

---

## Directory

```
├── camera                          Camera Pipeline packages
│   ├── camera_det3d
│   └── camera_pipeline
├── cam_lidar_bringup               System Bringup folder   **START HERE**
│   ├── configs
│   └── launch
├── data_tools                      Tools for converting and processing data
│   ├── data_tools
│   ├── launch
│   ├── scripts
│   └── src
├── Docs                            Additional information
│   ├── Archive
│   └── readme_images
├── drivers                         Hardware Drivers
│   ├── cam2image
    └── ouster_driver
├── fusion
│   ├── fusion_3d                   3D LiDAR + Camera Fusion module
│   └── fusion_module               2D LiDAR + Camera Fusion module
├── lidar
│   ├── learned_lidar_detector      Learned LiDAR Object Detection Module
│   ├── lidar_pipeline              Traditional LiDAR Object Detection Module
│   ├── obj_classifier              LiDAR Object Classification Module
│   └── pipeline_interfaces         Custom interfaces for LiDAR modules
├── Metrics                         Evaluation scripts and information
│   ├── 2D_HOTA
│   ├── 2D_mAP
│   ├── 3D_F1
│   └── Archive
├── obj_tracker                     Object Tracking Module
│   ├── obj_tracker
│   ├── resource
│   └── test
└── ros2_numpy                      Tools for converting msgs to numpy (Submodule)
    ├── ros2_numpy
    └── test
```
<!---
tree -d -L 2 -I __pycache__
--->

<!-- ## RQT Graph

![image](./Docs/readme_images/rosgraph.png) -->

---

# Setup

Tested on:

| Ubunbtu 20.04 |
|:-------------:|
|  i7-11800H @ 2.30GHz × 16|
|   32 GB RAM   |
|  NVIDIA Quadro T1200 |
| CUDA Version: 11.7 |

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

### 2.1 Python requirements

```
numpy, opencv, json, ultralytics
```

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

2. Run the launch file in the terminal `ros2 launch cam_lidar_bringup fusion_demo.launch.py`. The primary launch file will start all the nodes including Rviz2 for visualization. The launch file has differnt bringup options for different demonstrations and output.

3. Tune parameters and re-run. inspect the launch file to see what nodes are being executed. Sometime the launch file calls other launch files. Each node may have different paramters that can be adjusted.

---
# Troubleshooting and Comments

## Known issues

- cmake warning for PCL lib. Just re-build with the same command you used and the warning will be gone

## Colcon build fails?
- Check that the libraries you import are in your `CMakeLists.txt` (C++) or `setup.py` (Python) files.
- Certain warnings disappear if you build a 2nd time.
