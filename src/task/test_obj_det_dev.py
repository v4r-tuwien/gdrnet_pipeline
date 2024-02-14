#! /usr/bin/env python3
import rospy
from object_detector_msgs.srv import detectron2_service_server
from robokudo_msgs.msg import GenericImgProcAnnotatorAction, GenericImgProcAnnotatorResult, GenericImgProcAnnotatorFeedback, GenericImgProcAnnotatorGoal
import actionlib
from sensor_msgs.msg import Image, RegionOfInterest
import tf
import numpy as np

import cv2
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, Point, Quaternion

class PoseCalculator:
    def __init__(self):
        self.image_publisher = rospy.Publisher('/pose_estimator/image_with_roi', Image, queue_size=10)
        self.marker_publisher = rospy.Publisher('/object_markers', Marker, queue_size=10)
        self.bridge = CvBridge()
    
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

    # def publish_marker(self, result):
    #         marker = Marker()
    #         marker.header.frame_id = "camera_color_optical_frame"  # Replace with your desired frame ID
    #         marker.header.stamp = rospy.Time.now()
    #         marker.type = Marker.CUBE
    #         marker.action = Marker.ADD

    #         # Set the pose from the result
    #         # x is z

    #         marker.pose.position = Point(result.pose_results[0].position.x,
    #                                     result.pose_results[0].position.y,
    #                                     result.pose_results[0].position.z)
    #         marker.pose.orientation = Quaternion(result.pose_results[0].orientation.x,
    #                                             result.pose_results[0].orientation.y,
    #                                             result.pose_results[0].orientation.z,
    #                                             result.pose_results[0].orientation.w)

    #         size_x = ( 97.15 ) / 1000
    #         size_y = ( 66.62 ) / 1000
    #         size_z = ( 191.408 ) / 1000

    #         marker.scale.x = size_x
    #         marker.scale.y = size_y
    #         marker.scale.z = size_z

    #         # Set the color (green in this example)
    #         marker.color.r = 0.0
    #         marker.color.g = 1.0
    #         marker.color.b = 0.0
    #         marker.color.a = 0.5

    #         # Publish the marker
    #         self.marker_publisher.publish(marker)

    def publish_annotated_image(self, rgb, detections):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(rgb, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        for detection in detections:
            xmin = int(detection.bbox.ymin)
            ymin = int(detection.bbox.xmin)
            xmax = int(detection.bbox.ymax)
            ymax = int(detection.bbox.xmax)

            font_size = 1.0
            line_size = 3

            cv2.rectangle(cv_image, (xmin, ymin), (xmax, ymax), (0, 255, 0), line_size)

            class_name = detection.name
            score = detection.score
            label = f"{class_name}: {score:.2f}"
            cv2.putText(cv_image, label, (xmin, ymin - 20), cv2.FONT_HERSHEY_SIMPLEX, font_size, (0, 255, 0), line_size)

        # Publish annotated image
        annotated_image_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        self.image_publisher.publish(annotated_image_msg)

        # Display image for debugging
        # cv2.imshow("Annotated Image", cv_image)
        # cv2.waitKey(10)

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
        rate = rospy.Rate(2)  # Adjust the rate as needed (Hz)

        while not rospy.is_shutdown():
            # Assuming you have a way to get RGB and depth images
            rgb = rospy.wait_for_message(rospy.get_param('/pose_estimator/color_topic'), Image)
            depth = rospy.wait_for_message(rospy.get_param('/pose_estimator/depth_topic'), Image)

            print('Perform detection with YOLOv5 ...')
            detections = pose_calculator.detect(rgb)
            print("... received detection.")

            if detections is None or len(detections) == 0:
                print("nothing detected")
            else:
                print('Perform pose estimation with GDR-Net++ ...')
                try:
                    # Check for specific class and skip processing
                    if not any(detection.name == "036_wood_block" for detection in detections):
                        result = pose_calculator.estimate(rgb, depth, detections)
                        pose_calculator.publish_annotated_image(rgb, detections)
                        #pose_calculator.publish_marker(result)
                except Exception as e:
                    rospy.logerr(f"{e}")

            rate.sleep()

    except rospy.ROSInterruptException:
        pass

