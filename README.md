# Object detection and pose estimation pipeline using YOLOv5 and GDR-Net++
The pipeline is implemented to use YCB-V objects.

## Startup using the compose file(s)
[Configure](#configurations) all files first. Don't forget to set the [IP Adress of the ROS Master](#ros-master).

The following commands will download the necessary data and then build all the docker containers and start them. 

If the containers were already built before, you can still use the same commands (except download_data.sh) to start the pipeline.

densefusion + verefine:
```
./download_data.sh
cd compose
xhost +
docker-compose up
```

Three Docker containers will be started:
- yolov5: [YOLOv5](https://github.com/ultralytics/yolov5) trained on YCB-V Dataset
- gdrnetpp: Pose Estimation with [GDR-Net++](https://github.com/shanice-l/gdrnpp_bop2022)
- pose_estimator: Node that calls detect, estimate_pose service and delivers object poses

## Build Dockerfile with global context

yolov5:
`docker build -t yolov5 -f src/yolov5/Dockerfile .`

pose_estimator:
`docker build -t pose_estimator -f src/task/Dockerfile .`

gdrnpp:
`docker build -t gdrnpp -f src/gdrnpp/Dockerfile .`

## Visualization
In RVIZ you can view the final refined poses and the object segmentation by Mask-RCNN. 
They are published as images to these topics:
- ```/pose_estimator/segmentation```
- ```/pose_estimator/estimated_poses``` (densefusion)
 
## Service and Action Server
The pipeline will advertise a action server ```/pose_estimator/find_grasppose``` of the type [GenericImgProcAnnotator.action](https://github.com/v4r-tuwien/object_detector_msgs/blob/main/action/GenericImgProcAnnotator.action). The response represents the refined poses of the detected objects in the camera frame.

You can use the test file in ```src/test_obj_det.py``` to quickly check whether the pipeline works. 

The services that are internally called are 
- ```/pose_estimator/detect_objects``` of the type [detectron2_service_server.srv](https://github.com/v4r-tuwien/object_detector_msgs/blob/main/srv/detectron2_service_server.srv) 
- ```/pose_estimator/estimate_poses``` of the type [estimate_poses.srv](https://github.com/v4r-

### Main Action
```
#goal
sensor_msgs/Image rgb
sensor_msgs/Image depth
string description

---
#result
bool success
string result_feedback

# A list of bounding boxes for all detected objects
sensor_msgs/RegionOfInterest[] bounding_boxes

# Class IDs for each entry in bounding_boxes
int32[] class_ids

# Class confidence for each entry in bounding_boxes
float32[] class_confidences

# An image that can be used to send preprocessed intermediate results,
# inferred segmentation masks or maybe even a result image, depending on the use case
sensor_msgs/Image image

# The best pose for each entry in bounding_boxes
geometry_msgs/Pose[] pose_results

# Array-based string feedback when generating text for all detected objects etc.
string[] descriptions

---
#feedback
string feedback
```

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
The params for camera, model paths and topics are in config/params.yaml
Change this according to your project

```
im_width  # input image widht
im_height: # input image height
intrinsics:
- [538.391033533567, 0.0, 315.3074696331638]  # camera intrinsics
- [0.0, 538.085452058436, 233.0483557773859]
- [0.0, 0.0, 1.0]  

cfg_dir: "/verefine/data"

ycbv_names: /verefine/data/ycbv_names.json  # ycbv mapping form *.ply to object name
ycbv_verefine: /verefine/data/ycbv_verefine.json  # necessary verefine information about the models

color_topic: /camera/color/image_raw #  rgb image topic
depth_topic: /camera/depth/image_rect_raw  # depth image topic
camera_info_topic: /camera/color/camera_info # camera info topic
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
command: bash -c "source /yolov 5/catkin_ws/devel/setup.bash; ROS_NAMESPACE=pose_estimator python3 /yolov5/src/yolov5/ros_detection.py"
```

If you change it, the service names and visualization topics will change accordingly.

