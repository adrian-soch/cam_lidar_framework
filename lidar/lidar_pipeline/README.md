# lidar_pipeline

## Directory Structure

```
├── lidar_pipeline
│   ├── configs
│   ├── include
│   │   └── lidar_pipeline
│   ├── launch
│   └── src
```
---
## Setup

### Requirements

Only `sudo apt install ros-galactic-pcl-ros` must be installed separately, the rest of the depencies come with the ROS2 install.

---
## Build

Recommended addition arguments for `colcon build`:

```
colcon build --packages-select lidar_pipeline --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

---
## Usage

**Set params**

- Pipeline parameters can be adjusted in [lidar_pipeline_config.yaml](configs/lidar_pipeline_config.yaml).
- Location specefic data can be set in `configs/*_config.yaml`, an example is here: [configs/dec7_config.yaml](configs/dec7_config.yaml).


**Launch pipeline**

An example of a launch file that starts all the required nodes is [full_demo.launch.py](launch/full_demo.launch.py). This can be modified to start a rosbag or the Ouster driver node.

---
## Debug

Using VSCode, debugging a ROS2 node is easy.

Terminal 1:
```
colcon build --packages-select lidar_pipeline --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

ros2 run --prefix 'gdbserver localhost:3000' lidar_pipeline perception_node
```
In VSCode:
```
Create VSCode `launch.json`
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "ROS2 C++ Debugger",
            "request": "launch",
            "type": "cppdbg",
            "miDebuggerServerAddress": "localhost:3000",
            "cwd": "/",
            "program": "/home/adrian/dev/ros2_ws/install/lidar_pipeline/lib/lidar_pipeline/perception_node"
        }
    ]
}

# Run debugger in vscode
```
