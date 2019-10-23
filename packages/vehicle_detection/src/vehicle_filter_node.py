#!/usr/bin/env python
from cv_bridge import CvBridge, CvBridgeError
from duckietown_msgs.msg import VehicleCorners, VehiclePose, Pose2DStamped
from geometry_msgs.msg import Point32
from image_geometry import PinholeCameraModel
from mutex import mutex
from sensor_msgs.msg import CameraInfo
from math import sqrt, sin, cos
from std_msgs.msg import Float32
import cv2
import numpy as np
import os
import rospkg
import rospy
import threading
import yaml


class VehicleFilterNode(object):

    def __init__(self):
        self.node_name = rospy.get_name()
        self.bridge = CvBridge()
        self.backbumper = False #pw make parameter!!

        self.config = self.setupParam("~config", "baseline")
        if self.backbumper:
            self.cali_file_name = self.setupParam("~cali_file_name", "default")
        else:
            self.cali_file_name = self.setupParam("~cali_file_name", "LED_pattern")

        rospack = rospkg.RosPack()
        self.cali_file = rospack.get_path('duckietown') + \
            "/config/" + self.config + \
            "/vehicle_detection/vehicle_filter_node/" + \
            self.cali_file_name + ".yaml"
        if not os.path.isfile(self.cali_file):
            rospy.logwarn("[%s] Can't find calibration file: %s.\n"
                          % (self.node_name, self.cali_file))
        self.loadConfig(self.cali_file)

        self.sub_corners = rospy.Subscriber("~corners", VehicleCorners,
                                            self.processCorners, queue_size=1)

        self.pub_pose = rospy.Publisher("~pose", VehiclePose, queue_size=1)
        self.sub_info = rospy.Subscriber("~camera_info", CameraInfo,
                                         self.processCameraInfo, queue_size=1)
        self.pub_time_elapsed = rospy.Publisher("~pose_calculation_time",
                                                Float32, queue_size=1)
        self.pcm = PinholeCameraModel()
        rospy.loginfo("[%s] Initialization completed" % (self.node_name))
        self.lock = mutex()

    def setupParam(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value

    def loadConfig(self, filename):
        stream = file(filename, 'r')
        data = yaml.load(stream)
        stream.close()
        self.distance_between_centers = data['distance_between_centers']
        rospy.loginfo('[%s] distance_between_centers dim : %s' % (self.node_name,
                                                                  self.distance_between_centers))
        self.max_reproj_pixelerror_pose_estimation = data['max_reproj_pixelerror_pose_estimation']
        rospy.loginfo('[%s] max_reproj_pixelerror_pose_estimation : %s' % (self.node_name,
                                                                           self.max_reproj_pixelerror_pose_estimation))

    def processCameraInfo(self, camera_info_msg):
        if self.lock.testandset():
            self.pcm.fromCameraInfo(camera_info_msg)
            self.lock.unlock()

    def processCorners(self, vehicle_corners_msg):
        # do nothing - just relay the detection
        if self.lock.testandset():
            start = rospy.Time.now()
            # print(start)
            self.calcCirclePattern(vehicle_corners_msg.H,
                                   vehicle_corners_msg.W)
            points = []
            for Point32 in vehicle_corners_msg.corners:
                point = [Point32.x, Point32.y]
                points.append(point)
            points = np.array(points)
            # points = np.reshape(points, (2,-1))
            # print(points)
            # print(self.pcm.distortionCoeffs())
            if self.backbumper:
                (success, rotation_vector, translation_vector) = cv2.solvePnP(
                    self.circlepattern, points, self.pcm.intrinsicMatrix(), self.pcm.distortionCoeffs())

                if success:
                    points_reproj, _ = cv2.projectPoints(
                        self.circlepattern, rotation_vector, translation_vector, self.pcm.intrinsicMatrix(), self.pcm.distortionCoeffs())
                    error = 0
                    for i in range(0, len(points_reproj)):
                        error += cv2.norm(points[i],
                                          points_reproj[i, 0], cv2.NORM_L2)

                    mean_reproj_error = error / len(points_reproj)
                    # print(mean_reproj_error)
                    # print(self.max_reproj_pixelerror_pose_estimation)
            else:
                rotation_vector=[0,0,0]; #pw, valid assumption?

    			#f=318 #figure out how to get focal length of robot calibration
                f=self.pcm.intrinsicMatrix()[0,0] #check if this is 318
    			depth=self.distance_between_centers*f/abs(points[1,0]-points[0,0])
    			print("Depth: " +str(depth))
                imheight=480/2 #pw dont hardcode!!! is it even correct?
                imwidth=640
    			#imheight, imwidth = image_cv.shape[:2]
    			midt=(points[1,0]+points[0,0])/2-imwidth/2
    			Midt=midt/f*depth
    			print(Midt)
    			self.detected_log.append((Midt,depth))
                #implement log on shutdown
                # with open('/data/detected_duckiebot_log.csv', mode='w') as log:
        		# 	log = csv.writer(log, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        		# 	for data in self.detected_log:
    			# 	                log.writerow([data[0], data[1]])
    			#cv_image1=cv2.circle(cv_image1,(int(points[0,0]),int(points[0,1])), 5, (0,255,0), 2)
    			#cv_image1=cv2.circle(cv_image1,(int(points[1,0]),int(points[1,1])), 5, (0,255,0), 2)
                success = 1
                translation_vector=[Midt,depth,0] #pw coordinate system??
                translation_vector=translation_vector/np.linalg.norm(translation_vector) #normalize
                mean_reproj_error=0#pw cheating to fulfill next if condition



            if success:

                if mean_reproj_error < self.max_reproj_pixelerror_pose_estimation:
                    # print(translation_vector)
                    (R, jac) = cv2.Rodrigues(rotation_vector)
                    R_inv = np.transpose(R)
                    translation_vector = -np.dot(R_inv, translation_vector)
                    pose_msg_out = VehiclePose()
                    pose_msg_out.header.stamp = rospy.Time.now()
                    pose_msg_out.rho.data = sqrt(
                        translation_vector[2] ** 2 + translation_vector[0] ** 2)
                    pose_msg_out.psi.data = np.arctan2(
                        -R_inv[2, 0], sqrt(R_inv[2, 1] ** 2 + R_inv[2, 2] ** 2))
                    pose_msg_out.detection.data = vehicle_corners_msg.detection.data
                    R2 = np.array([[cos(pose_msg_out.psi.data), -sin(pose_msg_out.psi.data)], [
                                  sin(pose_msg_out.psi.data), cos(pose_msg_out.psi.data)]])
                    translation_vector = - \
                        np.array([translation_vector[2],
                                  translation_vector[0]])
                    translation_vector = np.dot(
                        np.transpose(R2), translation_vector)
                    pose_msg_out.theta.data = np.arctan2(
                        translation_vector[1], translation_vector[0])
                    self.pub_pose.publish(pose_msg_out)
                else:
                    rospy.loginfo(
                        "[%s] Pose estimation failed, too high reprojection error." % (self.node_name))
            else:
                rospy.loginfo("[%s] Pose estimation failed." %
                              (self.node_name))

            elapsed_time = (rospy.Time.now() - start).to_sec()
            self.pub_time_elapsed.publish(elapsed_time)
        self.lock.unlock()
        return

    def calcCirclePattern(self, height, width):
        self.circlepattern_dist = self.distance_between_centers
        self.circlepattern = np.zeros([height * width, 3])
        for i in range(0, width):
            for j in range(0, height):
                self.circlepattern[i + j * width, 0] = self.circlepattern_dist * \
                    i - self.circlepattern_dist * (width - 1) / 2
                self.circlepattern[i + j * width, 1] = self.circlepattern_dist * \
                    j - self.circlepattern_dist * (height - 1) / 2
        # print(self.circlepattern)


if __name__ == '__main__':
    rospy.init_node('vehicle_filter_node', anonymous=False)
    vehicle_filter_node = VehicleFilterNode()
    rospy.spin()
