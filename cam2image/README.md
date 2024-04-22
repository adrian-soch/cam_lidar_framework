# Cam2Image

```
├── cam2image
│   ├── include
│   │   └── cam2image
│   ├── launch
│   └── src
```

This ROS 2 node get images from a USB webcam and publishes them at the specifided rate. See [cam_stream.cpp](src/cam_stream.cpp) for details about the parameters, or look at [camera_launch.py](launch/camera_launch.py) for example parameters.

---
## Use

Build the package with colcon, use ros run or ros launch to start the node. To help find your device ID:

```
sudo apt-get install v4l-utils
v4l2-ctl --list-devices

# Output example
# Logitech BRIO (usb-0000:00:14.0-6):
#	/dev/video2
#	/dev/video3
#	/dev/video4
#	/dev/video5
```

---

## Deveopment

[cam_stream.cpp](src/cam_stream.cpp) can be modified to perform processing with OpenCV or other libraries before publishing. But, you should consider using a seperate node and leavong this as is.

Furthermore, other C++ nodes which consume the published image could benefit from being placed in the same container as this driver. Zero copy transport can be leveraged to increase performance.
