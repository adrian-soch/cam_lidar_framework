# Sensor Setup

## Ouster LiDAR Setup

CMHT Model is OS-1 64

### Ouster ROS2 Driver

Install via `sudo apt install -y ros-galactic-ros2-ouster`. This installs the driver and the custom messages.

### Connect Hardware

Ethernet and power into the box. Cable from box to LiDAR. Plug into an outlet, ethernet to Laptop.

### Initial Setup

- Turn off Wifi.
- In Ethernet settings. Set ethernet device to static IP 10.5.5.90, netmask is 24, Gateway empty.
- Open `os1-991920100080.local` in browser.
- Find the configuration tab, confirm the IP of the *UDP Destination Address* to 10.5.5.90 
- Copy param and launch file from here [https://github.com/ros-drivers/ros2\_ouster\_drivers/tree/ros2/ros2_ouster](https://github.com/ros-drivers/ros2_ouster_drivers/tree/ros2/ros2_ouster)
    

### Optional - Update Firmware

Minimum V2.1 required. Currently only V2.4 is available from Ouster website.
1.  Visit https://ouster.com/downloads/ and download the desired firmware image.
2.  Go to the LiDAR page same as in [Initial Setup](#initial-setup), and upload the new firmware.
3.  Wait for the sensor to reboot itself (this takes a few minutes for it to stop spinning and begin spinning again).

## Logitech Brio Webcam

To list availbale webcams on the device: 
```
sudo apt-get install v4l-utils
v4l2-ctl --list-devices

# Output example
Logitech BRIO (usb-0000:00:14.0-6):
	/dev/video2
	/dev/video3
	/dev/video4
	/dev/video5
```

Here video2 is the main camera, video4 is the IR camera, 3 and 5 are other camera metadata (not used by us).

# ROS2 Stuff

#### Renaming topics (static)
- Static renaming of topics example, remap /Camera to /image:
	`python3 camera_processing_node.py --ros-args -r Camera:=image`
#### QoS
-  `ros2 topic info /topic_name --verbose`
#### Rosbag2
- **Loop bag** playback `ros2 bag play -l <path_to_bag>`
- **Pause/Play bag** playbag `ros2 service call /rosbag2_player/toggle_paused rosbag2_interfaces/TogglePaused`
- **To play 1 message** (need to call once per # of messages you have) `ros2 service call /rosbag2_player/play_next rosbag2_interfaces/PlayNext`
More info [here](https://docs.ros.org/en/humble/Releases/Release-Galactic-Geochelone.html).

# Recording Lidar data
- `ros2 bag record /points -o ~/bags/ousterTest4_nov21 --max-cache-size 280`
	- Need to increase the cache size in order to keep up with the data, otherwise nothing looks right
- `ros2 bag record /points /image /tf_static -o ~/dev/bags/brioAndOusterTest_0 --max-cache-size 500`
	- no visible lag

# Intrisic Sensor Calibration

The LiDAR sensor is pre-calibrated in the factory. For the Ouster OS1 model, refer to the hardware manual for querying the calibrations parameters via TCP.

Each camera has a different intrisic calibration in order to rectify the resultant image. This is due to the warping and distortion caused by variation between individual image sensors. A common calibration procedure involves a checkboard pattern of know dimension, multiple images are taken by the camera with the checkerboard in multiplte orientations. The edges and cprners are detected, and with the assumtion that the board has perfect lines, the distortion within the image can be calculated. OpenCV provides a convientment method of camera calibration.

### Calibration in ROS2
For camera calibration, follow this [tutorial](https://navigation.ros.org/tutorials/docs/camera_calibration.html) and [here](https://jeffzzq.medium.com/ros2-image-pipeline-tutorial-3b18903e7329). It explains the parameters and process that you will follow. Clone the following repos into `<ROS2_WS>/src` and then `cd ..` then `colcon_build`.
 - https://github.com/ros-perception/image_pipeline/tree/galactic **(change to galactic branch)**

**Terminal 1 (with ros2 sourced)**
```
# device_id is 2 for video2
ros2 run image_tools cam2image --ros-args -p device_id:=2 -p frequency:=10.0 -p width:=1920 -p height:=1080
```

**Terminal 2 (with ros2 sourced)**
```
cd <ROS2_WS>
. install/setup.bash

# Adjust size and square based on your chessboard pattern
ros2 run camera_calibration cameracalibrator --size=7x7 --square=0.024 --ros-args

# Optional - copy result somewhere
cp /tmp/calibrationdata.tar.gz <desired_path>/calibrationdata.tar.gz

```

# Extrinsic Sensor Calibration

### Theory
This describes how to determine the homogenus transformation matrix from the LiDAR frame to the Camera Frame. Firstly we must detemrine the orientation and location of each sensor origin. This was done in CAD and using the information from Ouster and OpenCV for the LiDAR and camera/image respectivly.

**Ouster Lidar Coordinate Frame**: Info [here](https://static.ouster.dev/sensor-docs/image_route1/image_route2/sensor_data/sensor-data.html). Sensor coordinate frame is the one we are using.
**Image Coordinate Frame**: Info [here](https://docs.opencv.org/4.6.0/d9/d0c/group__calib3d.html).

To transform the LiDAR coordinate frame to the camera frame we establish the x/y/z triad in the lidar CAD and the camera CAD. This allows us to determine the rotational transformation by inspection.