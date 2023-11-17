#! /usr/bin/env python3
import rospy
from object_detector_msgs.srv import detectron2_service_server, estimate_poses
import actionlib
from sensor_msgs.msg import Image
import tf

def detect(rgb):
    rospy.wait_for_service('detect_objects')
    try:
        detect_objects = rospy.ServiceProxy('detect_objects', detectron2_service_server)
        response = detect_objects(rgb)
        return response.detections.detections
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

def estimate(rgb, depth, detection):
    rospy.wait_for_service('estimate_poses')
    try:
        estimate_pose = rospy.ServiceProxy('estimate_poses', estimate_poses)
        response = estimate_pose(detection, rgb, depth)
        return response.poses
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

def get_poses():
    rate = rospy.Rate(2) # Hz
    while not rospy.is_shutdown():
        rgb = rospy.wait_for_message(rospy.get_param('/pose_estimator/color_topic'), Image)
        # TODO: In the current implementation we do not use the depth, so we do not have to describe to it
        # depth = rospy.wait_for_message(rospy.get_param('/pose_estimator/depth_topic'), Image)
        depth = rgb
        print('Perform detection with YOLOv5 ...')
        detections = detect(rgb)
        print("... received detection.")

        if detections is None or len(detections) == 0:
            print("nothing detected")
        
        else:
            print('Perform pose estimation with GDR-Net++ ...')
            for detection in detections:
                instance_poses = estimate(rgb, depth, detection)
                print(f"Got pose for: {instance_poses}")

        rate.sleep()
    
if __name__ == "__main__":
    rospy.init_node("get_poses")

    get_poses()

