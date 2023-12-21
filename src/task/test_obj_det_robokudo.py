#! /usr/bin/env python3
import rospy
from object_detector_msgs.srv import detectron2_service_server
from robokudo_msgs.msg import GenericImgProcAnnotatorAction, GenericImgProcAnnotatorResult, GenericImgProcAnnotatorFeedback, GenericImgProcAnnotatorGoal
import actionlib
from sensor_msgs.msg import Image, RegionOfInterest
import tf
import numpy as np

class PoseCalculator:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('/pose_estimator/gdrnet', 
                                                   GenericImgProcAnnotatorAction)
        self.client.wait_for_server()

        self.server = actionlib.SimpleActionServer('/pose_estimator', 
                                                   GenericImgProcAnnotatorAction, 
                                                   execute_cb=self.get_poses_robokudo, 
                                                   auto_start=False)
        self.server.start()

    def detect(self, rgb):
        rospy.wait_for_service('detect_objects')
        try:
            detect_objects = rospy.ServiceProxy('detect_objects', detectron2_service_server)
            response = detect_objects(rgb)
            return response.detections.detections
        except rospy.ServiceException as e:

            print("Service call failed: %s" % e)

    def estimate(self, rgb, depth, detections):
        try:
            goal = GenericImgProcAnnotatorGoal()
            goal.rgb = rgb
            goal.depth = depth
            bb_detections = []
            class_names = []
            description = ''
            ind_detections = np.arange(0, len(detections), 1)
            for detection, index in zip(detections, ind_detections):
                print(f"{index=}")
                roi = RegionOfInterest()
                roi.x_offset = detection.bbox.ymin
                roi.y_offset = detection.bbox.xmin
                roi.height = detection.bbox.xmax - detection.bbox.xmin
                roi.width = detection.bbox.ymax - detection.bbox.ymin
                roi.do_rectify = False

                bb_detections.append(roi)
                class_names.append(detection.name)
                if index == 0:
                    description = description + f'"{detection.name}": "{detection.score}"'
                else:
                    description = description + f', "{detection.name}": "{detection.score}"'
            description = '{' + description + '}'
            print(f"{description=}")
            goal.bb_detections = bb_detections
            goal.class_names = class_names
            goal.description = description
            self.client.send_goal(goal)
            self.client.wait_for_result()
            result = self.client.get_result()
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

        return result

    def get_poses_robokudo(self, goal):
        res = GenericImgProcAnnotatorResult()
        res.success = False
        res.result_feedback = "calculated: "
        feedback = GenericImgProcAnnotatorFeedback()

        # === check if we have an image ===
        if goal.rgb is None or goal.depth is None:
            print("no images available")
            res.result_feedback = "no images available"
            res.success = False
            self.server.set_succeeded(res)
            # self.server.set_preempted()
            return
        rgb = goal.rgb
        depth = goal.depth
        print('Perform detection with YOLOv5 ...')
        detections = self.detect(rgb)
        print("... received detection.")

        if detections is None or len(detections) == 0:
            print("nothing detected")
            self.server.set_aborted(res)
            return
        else:
            print('Perform pose estimation with GDR-Net++ ...')
            try:
                res = self.estimate(rgb, depth, detections)
            except Exception as e:
                rospy.logerr(f"{e=}")

        # res.class_names = class_names
        res.result_feedback = res.result_feedback + ", class_names"
        # res.class_confidences = class_confidences
        res.result_feedback = res.result_feedback + ", class_confidences"
        # res.pose_results = pose_results
        res.result_feedback = res.result_feedback + ", pose_results"
        res.success = True
        print(f"{res=}")
        self.server.set_succeeded(res)

    
if __name__ == "__main__":
    rospy.init_node("calculate_poses")
    try:
        pose_calculator = PoseCalculator()
    except Exception as e:
        print(f"{e}")
    rospy.spin()

