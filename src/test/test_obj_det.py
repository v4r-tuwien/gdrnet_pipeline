#! /usr/bin/env python3
import rospy
from robokudo_msgs.msg import GenericImgProcAnnotatorAction, GenericImgProcAnnotatorGoal
import actionlib
from sensor_msgs.msg import Image


def get_poses():
    client = actionlib.SimpleActionClient('/pose_estimator', GenericImgProcAnnotatorAction)
    res = client.wait_for_server(rospy.Duration(10.0))
    if res is False:
        rospy.logerr('Timeout when trying to connect to actionserver')
        return
    goal = GenericImgProcAnnotatorGoal()
    print('Waiting for images')
    rgb = rospy.wait_for_message('/hsrb/head_rgbd_sensor/rgb/image_rect_color', Image)
    depth = rospy.wait_for_message('/hsrb/head_rgbd_sensor/depth_registered/image_rect_raw', Image)
    print('Sending Goal')
    goal.rgb = rgb
    goal.depth = depth
    client.send_goal(goal)
    client.wait_for_result()
    # print(f"Got pose for: {client.get_result().descriptions}")
    print(f"Got pose for: {client.get_result().pose_results}")

if __name__ == "__main__":
    rospy.init_node("get_poses")
    get_poses()

