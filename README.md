# GDR-Net Pipeline for Object Detection and Pose Estimation with ROS
The pipeline is implemented for the detection and pose estimation of YCB-V, and the YCB-iChores objects.
This repo includes submodules.
Clone this repository via either SSH or HTTPS and clone the submodules as well by:
- `git clone https://github.com/v4r-tuwien/gdrnet_pipeline.git`
- `cd gdrnet_pipeline`
- `git submodule init`
- `git submodule update`

Use the following to pull updates:
- `git pull --recurse-submodules`

The YOLOv8 and the GDRN++ docker containers need ~35 GB of free disk space.

## Requirements
- The YOLOv8 and the GDRN++ docker containers need ~35 GB of free disk space.
- A NVIDIA GPU with >16GB is recommended
- System RAM >= 64 GB is recommended

| Module                               | VRAM Usage | System RAM Usage | Disk Space | Description                                 | Link                                                                                   |
|--------------------------------------|------------|------------------|------------|---------------------------------------------|----------------------------------------------------------------------------------------|
| YOLOv8                               | 2.2 GB     | 1.4 GB           | 16.2 GB    | 2D Object Detection with YOLOv8             | [YOLOv8](https://github.com/hoenigpeter/yolov8_ros)                                    |
| GDRN++                               | 3.5 GB     | 7.0 GB           | 18.7 GB    | 6D Object Pose Estimation with GDRN++       | [GDR-Net++](https://github.com/hoenigpeter/gdrnpp_bop2022)                             |


## ROS Config
ROS_IP and ROS_MASTER_URI differ from setup to setup.
### Local Machine Debugging
When doing debugging on a local machine with a local ROS master e.g. with https://github.com/hoenigpeter/realsense_ros_docker the ROS_IP and ROS_MASTER_URI are:
- ROS_IP=127.0.0.1
- ROS_MASTER_URI=http://127.0.0.1:11311

### Robot Setup
This is different when the roscore is running on a robot, e.g. HSR:
- ROS_MASTER_URI=http://10.0.0.143:11311
- ROS_IP=10.0.0.232

## Startup using the compose file(s)
[Configure](#configurations) all files first. Don't forget to set the [IP Adress of the ROS Master](#ros-master) if you have another ROS-Core running.

The following commands will download the necessary data and then build all the docker containers and start them. 

If the containers were already built before, you can still use the same commands (except download_data.sh) to start the pipeline.

full pipeline:
```
./download_data.sh
cd compose/pipeline
xhost +
```
then for YCB-V dataset objects (ROS example for local setup, adapt ROS_MASTER_URI and ROS_IP depending on your setup):

```
ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1  DATASET=ycbv CONFIG=params_realsense.yaml docker-compose up
```
or for YCB-iChores dataset objects:

```
ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1  DATASET=ycb_ichores CONFIG=params_realsense.yaml docker-compose up
```
Docker containers for yolov8, GDRN++ and MediaPipe will be started.

## Visualization
To visualize the estimated bounding box and object pose use RViZ and load the RViZ config from ./configs/default.rviz
In RViZ visualize the following topics:
- RGB image with bounding box: /pose_estimator/image_with_roi
- Marker of estimated mesh: /object_markers
- Activate visualization of TFs for 6DoF frame of estimated object pose

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

color_topic: /hsrb/head_rgbd_sensor/rgb/image_rect_color #  rgb image topic
depth_topic: /hsrb/head_rgbd_sensor/depth_registered/image_rect_raw  # depth image topic

color_frame_id: head_rgbd_sensor_rgb_frame
```

### ROS Master
The ROS Master is set in the docker-compose.yml file for each container 
```
environment:
      ROS_MASTER_URI: "http://hsrb.local:11311"
      ROS_IP: "10.0.0.232"
```

