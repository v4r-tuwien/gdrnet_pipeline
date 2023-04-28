#! /usr/bin/env python3
import rospy
from robokudo_msgs.msg import GenericImgProcAnnotatorAction, GenericImgProcAnnotatorGoal
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

def get_poses():
    client = actionlib.SimpleActionClient('/pose_estimator/find_grasppose', GenericImgProcAnnotatorAction)
    res = client.wait_for_server(rospy.Duration(10.0))
    if res is False:
        rospy.logerr('Timeout when trying to connect to actionserver')
        return
    goal = GenericImgProcAnnotatorGoal()
    print('Waiting for images')
    while(1):
        rgb = rospy.wait_for_message('/camera/color/image_raw', Image)
        depth = rospy.wait_for_message('/camera/depth/image_rect_raw', Image)
        print('Sending Goal')
        goal.rgb = rgb
        goal.depth = depth

        print("Received image with: ", rgb.width, " ", rgb.height)
        client.send_goal(goal)
        client.wait_for_result()
        result = client.get_result()
        print(f"Got pose for: {result.pose_results}")
    
if __name__ == "__main__":
    rospy.init_node("get_poses")

    get_poses()

