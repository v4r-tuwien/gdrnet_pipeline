# Object detection and pose estimation pipeline using YOLOv5 and GDR-Net++
The pipeline is implemented for the detection and pose estimatino of YCB-V objects.
This repo includes submodules.
Clone this repository via either SSH or HTTPS and clone the submodules as well by:
- `git clone https://github.com/v4r-tuwien/gdrnet_pipeline.git`
- `cd gdrnet_pipeline`
- `git submodule init`
- `git submodule update`

Use the following to pull updates:
- `git pull --recurse-submodules`

Currently the YOLOv5 and the GDRN++ docker containers need ~35 GB of free disk space.
We try to downsize in the future.

## Startup using the compose file(s)
[Configure](#configurations) all files first. Don't forget to set the [IP Adress of the ROS Master](#ros-master) if you have another ROS-Core running.

The following commands will download the necessary data and then build all the docker containers and start them. 

If the containers were already built before, you can still use the same commands (except download_data.sh) to start the pipeline.

full pipeline:
```
./download_data.sh
cd compose/pipeline
xhost +
docker-compose up
```

Three Docker containers will be started:
- yolov5: [YOLOv5](https://github.com/ultralytics/yolov5) trained on YCB-V Dataset
- gdrnetpp: Pose Estimation with [GDR-Net++](https://github.com/shanice-l/gdrnpp_bop2022) trained on YCB-V Dataset
- task: Node that calls detect, estimate_pose service and calculates object poses

## ROS Service Calls
This package hosts two main services:
- ```/pose_estimator/detect_objects``` of the type [detectron2_service_server.srv](https://github.com/v4r-tuwien/object_detector_msgs/blob/main/srv/detectron2_service_server.srv) 
- ```/pose_estimator/estimate_poses``` of the type [estimate_poses.srv](https://github.com/v4r-tuwien/object_detector_msgs/blob/main/srv/estimate_poses.srv)

You can directly use the services in your own nodes.
The services are also called using the task container which is automatically started via `docker compose up`.

### Main Service

#### estimate_poses.srv
```
Detection det
sensor_msgs/Image rgb
sensor_msgs/Image depth
---
PoseWithConfidence[] poses
```

### Important Messages
#### PoseWithConfidence.msg
```
string name
geometry_msgs/Pose pose
float32 confidence
```

#### Detecions.msg
```
Header header

uint32 height
uint32 width

Detection[] detections
```

#### Detection.msg
```
string name
float32 score
BoundingBox bbox
int64[] mask
```

## Configurations
### Config-Files
The params for camera intrinsics and rgb/depth-topics are in config/params.yaml
Change this according to your project
Currently the resolution must be 640x480 
```
im_width  # input image widht
im_height: # input image height
intrinsics:
- [538.391033533567, 0.0, 315.3074696331638]  # camera intrinsics
- [0.0, 538.085452058436, 233.0483557773859]
- [0.0, 0.0, 1.0] 

color_topic: /camera/color/image_raw #  rgb image topic
depth_topic: /camera/depth/image_rect_raw  # depth image topic

color_frame_id: camera_color_optical_frame
```

### ROS Master
The ROS Master is set in the docker-compose.yml file for each container 
```
environment:
      ROS_MASTER_URI: "http://127.0.0.1:11311"
      ROS_IP: "127.0.0.1"
```
### ROS Namespace
The Namespace is also defined in the docker-compose.yml file for each container. It is passed as command with the python script calls like this:
```
command: bash -c "source /yolo/catkin_ws/devel/setup.bash; ROS_NAMESPACE=pose_estimator python /yolov5/src/yolov5/ros_detection.py"
```

If you change it, the service names and visualization topics will change accordingly.

