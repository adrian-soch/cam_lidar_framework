# Object Classifier

```
.
├── lidar_obj_classifier
│   └── server
├── resource
└── test

```

lidar_obj_classifier contains the python scripts for performing object classification based on the published 3d Detections and an array of pointclouds. Each detection is paired with a pointcloud of that object, features are extracted and class predictions are calculated.

The `server` folder contains an the server side implemention of a ROS2 service for inferencing the classifier. It is not used but stored here in case it is needed later.
