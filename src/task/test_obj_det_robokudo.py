#! /usr/bin/env python3
import rospy
from object_detector_msgs.srv import detectron2_service_server, estimate_poses
from robokudo_msgs.msg import GenericImgProcAnnotatorAction, GenericImgProcAnnotatorResult, GenericImgProcAnnotatorFeedback
import actionlib
from sensor_msgs.msg import Image
from object_detector_msgs.msg import PoseWithConfidence
import tf

class PoseCalculator:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('/pose_estimator/gdrnet', 
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

    def estimate(self, rgb, depth, detection):
        rospy.wait_for_service('estimate_poses')
        try:
            estimate_pose = rospy.ServiceProxy('estimate_poses', estimate_poses)
            response = estimate_pose(detection, rgb, depth)
            print(f"{response=}")
            return response.poses
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

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
            class_names = []
            class_confidences = []
            pose_results = []
            # print(f"{detections=}")
            for detection in detections:
                instance_poses_list:PoseWithConfidence = self.estimate(rgb, depth, detection)
                instance_poses = instance_poses_list[0]
                # print(f"Got pose for: {instance_poses}")
                if len(instance_poses_list) >= 1:
                    #TODO fill message -> pose/class_name und ggf. confidence
                    class_names.append(instance_poses.name)
                    class_confidences.append(instance_poses.confidence)
                    pose_results.append(instance_poses.pose)

        res.class_names = class_names
        res.result_feedback = res.result_feedback + ", class_names"
        res.class_confidences = class_confidences
        res.result_feedback = res.result_feedback + ", class_confidences"
        res.pose_results = pose_results
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

