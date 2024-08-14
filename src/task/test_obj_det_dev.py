#! /usr/bin/env python3
import rospy
from object_detector_msgs.srv import detectron2_service_server, estimate_pointing_gesture
from robokudo_msgs.msg import GenericImgProcAnnotatorAction, GenericImgProcAnnotatorResult, GenericImgProcAnnotatorFeedback, GenericImgProcAnnotatorGoal
import actionlib
from sensor_msgs.msg import Image, RegionOfInterest
import tf
import numpy as np
import open3d as o3d
import yaml
import os
import time

import cv2
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, Point, Quaternion

import argparse

class PoseCalculator:
    def __init__(self, dataset="ycbv.yaml"):
        self.image_publisher = rospy.Publisher('/pose_estimator/image_with_roi', Image, queue_size=10)
        self.bridge = CvBridge()

        self.models = self.load_models("/root/task/datasets/" + dataset + "/models", "/root/config/" + dataset + ".yaml")

        self.client = actionlib.SimpleActionClient('/pose_estimator/gdrnet', 
                                                   GenericImgProcAnnotatorAction)
        self.client.wait_for_server()

        self.server = actionlib.SimpleActionServer('/pose_estimator',
                                                   GenericImgProcAnnotatorAction,
                                                   execute_cb=self.get_poses_robokudo,
                                                   auto_start=False)

        self.frame_id = rospy.get_param('/pose_estimator/color_frame_id')

        self.server.start()

    def load_models(self, folder_path, yaml_file_path):
        with open(yaml_file_path, 'r') as yaml_file:
            yaml_data = yaml.safe_load(yaml_file)
        
        names = yaml_data.get('names', {})
        models = {}

        for obj_id, obj_name in names.items():
            filename = f"obj_{int(obj_id):06d}.ply"
            model_path = os.path.join(folder_path, filename)
            if os.path.exists(model_path):
                model = o3d.io.read_point_cloud(model_path)
                vertices = np.asarray(model.points)
                colors = np.asarray(model.colors) if model.colors else None
                models[obj_name] = {'vertices': vertices, 'colors': colors}
        return models

    def detect_objects(self, rgb):
        rospy.wait_for_service('detect_objects')
        try:
            detect_objects_service = rospy.ServiceProxy('detect_objects', detectron2_service_server)
            response = detect_objects_service(rgb)
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
            goal.bb_detections = bb_detections
            goal.class_names = class_names
            goal.description = description
            self.client.send_goal(goal)
            self.client.wait_for_result()
            result = self.client.get_result()
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

        return result

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
        print('Perform detection with YOLOv8 ...')
        detections = self.detect_objects(rgb)
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

    def publish_mesh_marker(self, cls_name, quat, t_est):
        from visualization_msgs.msg import Marker
        vis_pub = rospy.Publisher("/gdrnet_meshes", Marker, latch=True)
        model_data = self.models.get(cls_name, None)
        model_vertices = np.array(model_data['vertices'])/1000
        #model_colors = model_data['colors']

        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.TRIANGLE_LIST
        marker.ns = cls_name
        marker.action = Marker.ADD
        marker.pose.position.x = t_est[0]
        marker.pose.position.y = t_est[1]
        marker.pose.position.z = t_est[2]
        #quat = Rotation.from_matrix(R_est).as_quat()
        marker.pose.orientation.x = quat[0]
        marker.pose.orientation.y = quat[1]
        marker.pose.orientation.z = quat[2]
        marker.pose.orientation.w = quat[3]
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        from geometry_msgs.msg import Point
        from std_msgs.msg import ColorRGBA
        #assert model_vertices.shape[0] == model_colors.shape[0]

        # TRIANGLE_LIST needs 3*x points to render x triangles 
        # => find biggest number smaller than model_vertices.shape[0] that is still divisible by 3
        shape_vertices = 3*int((model_vertices.shape[0] - 1)/3)
        for i in range(shape_vertices):
            pt = Point(x = model_vertices[i, 0], y = model_vertices[i, 1], z = model_vertices[i, 2])
            marker.points.append(pt)
            rgb = ColorRGBA(r = 1, g = 0, b = 0, a = 1.0)
            marker.colors.append(rgb)

        vis_pub.publish(marker)

def parse_opt():
    parser = argparse.ArgumentParser()
    parser.add_argument('--dataset', type=str, default='ycbv.yaml')

    opt = parser.parse_args()
    return opt

if __name__ == "__main__":
    rospy.init_node("calculate_poses")
    opt = parse_opt()

    try:
        pose_calculator = PoseCalculator(**vars(opt))
        rate = rospy.Rate(10)  # Adjust the rate as needed (Hz)

        while not rospy.is_shutdown():
            # Assuming you have a way to get RGB and depth images
            rgb = rospy.wait_for_message(rospy.get_param('/pose_estimator/color_topic'), Image)
            depth = rospy.wait_for_message(rospy.get_param('/pose_estimator/depth_topic'), Image)

            #print('Perform detection with YOLOv8 ...')
            t0 = time.time()
            detections = pose_calculator.detect_objects(rgb)
            time_detections = time.time() - t0

            pose_calculator.publish_annotated_image(rgb, detections)
            #print("... received object detection.")

            estimated_poses = []
            t0 = time.time()
            if detections is None or len(detections) == 0:
                print("nothing detected")
            else:
                #print('Perform pose estimation with GDR-Net++ ...')
                try:
                    # Check for specific class and skip processing
                    if not any(detection.name == "036_wood_block" for detection in detections):
                        estimated_poses = pose_calculator.estimate(rgb, depth, detections)
                        print(estimated_poses.class_names[0] + " with confidence " + str(estimated_poses.class_confidences[0]))
                        #pose_calculator.publish_marker(result)

                except Exception as e:
                    rospy.logerr(f"{e}")
            time_object_poses = time.time() - t0

            # Print the timed periods
            print(f"Time for object detection: {time_detections:.2f} seconds")
            print(f"Time for object pose estimation: {time_object_poses:.2f} seconds")
            print()

    except rospy.ROSInterruptException:
        pass

