#!/usr/bin/env python
from copy import deepcopy
from cv_bridge import CvBridge, CvBridgeError
from duckietown_msgs.msg import BoolStamped, VehicleCorners
from geometry_msgs.msg import Point32
from mutex import mutex
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float32
import cv2
import numpy as np
import os
import rospkg
import rospy
import threading
import time
import yaml


class VehicleDetectionNode(object):

    def __init__(self):
        self.node_name = rospy.get_name()
        self.bridge = CvBridge()
        self.active = True
        self.backbumper = False #pw
        self.config = self.setupParam("~config", "baseline")
        self.cali_file_name = self.setupParam("~cali_file_name", "default")
        self.publish_freq = self.setupParam("~publish_freq", 2.0)
        self.publish_duration = rospy.Duration.from_sec(1.0/self.publish_freq)
        self.last_stamp = rospy.Time.now()
        rospack = rospkg.RosPack()
        self.cali_file = rospack.get_path('duckietown') + \
            "/config/" + self.config + \
            "/vehicle_detection/vehicle_detection_node/" +  \
            self.cali_file_name + ".yaml"
        if not os.path.isfile(self.cali_file):
            rospy.logwarn("[%s] Can't find calibration file: %s.\n"
                          % (self.node_name, self.cali_file))
        self.loadConfig(self.cali_file)

        self.lock = mutex()
        self.sub_image = rospy.Subscriber("~image", CompressedImage,
                                          self.processImage, buff_size=921600,
                                          queue_size=1)
        self.sub_switch = rospy.Subscriber("~switch", BoolStamped,
                                           self.cbSwitch, queue_size=1)
        self.pub_detection = rospy.Publisher("~detection",
                                             BoolStamped, queue_size=1)
        self.pub_corners = rospy.Publisher("~corners",
                                           VehicleCorners, queue_size=1)
        self.pub_circlepattern_image = rospy.Publisher("~circlepattern_image",
                                                       Image, queue_size=1)
        self.pub_time_elapsed = rospy.Publisher("~detection_time",
                                                Float32, queue_size=1)

        rospy.loginfo("[%s] Initialization completed" % (self.node_name))

    def setupParam(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value

    def loadConfig(self, filename):
        stream = file(filename, 'r')
        data = yaml.load(stream)
        stream.close()
        self.circlepattern_dims = tuple(data['circlepattern_dims']['data'])
        self.blobdetector_min_area = data['blobdetector_min_area']
        self.blobdetector_min_dist_between_blobs = data['blobdetector_min_dist_between_blobs']
        self.publish_circles = data['publish_circles']
        rospy.loginfo('[%s] circlepattern_dim : %s' % (self.node_name,
                                                       self.circlepattern_dims,))
        rospy.loginfo('[%s] blobdetector_min_area: %.2f' % (self.node_name,
                                                            self.blobdetector_min_area))
        rospy.loginfo('[%s] blobdetector_min_dist_between_blobs: %.2f' % (self.node_name,
                                                                          self.blobdetector_min_dist_between_blobs))
        rospy.loginfo('[%s] publish_circles: %r' % (self.node_name,
                                                    self.publish_circles))

    def cbSwitch(self, switch_msg):
        self.active = True #switch_msg.data, changed pw!

    def processImage(self, image_msg):

        if not self.active:
            return

        now = rospy.Time.now()
        if now - self.last_stamp < self.publish_duration:
            return
        else:
            self.last_stamp = now


        vehicle_detected_msg_out = BoolStamped()
        vehicle_corners_msg_out = VehicleCorners()
        try:
            image_cv = self.bridge.compressed_imgmsg_to_cv2(
                image_msg, "bgr8")
        except CvBridgeError as e:
            print e

        start = rospy.Time.now()
        if self.backbumper:
            params = cv2.SimpleBlobDetector_Params()
            params.minArea = self.blobdetector_min_area
            params.minDistBetweenBlobs = self.blobdetector_min_dist_between_blobs
            simple_blob_detector = cv2.SimpleBlobDetector_create(params)
            (detection, corners) = cv2.findCirclesGrid(image_cv,
                                                        self.circlepattern_dims, flags=cv2.CALIB_CB_SYMMETRIC_GRID,
                                                        blobDetector=simple_blob_detector)
            if detection:
                # print(corners)
                points_list = []
                for point in corners:
                    corner = Point32()
                    # print(point[0])
                    corner.x = point[0, 0]
                    # print(point[0,1])
                    corner.y = point[0, 1]
                    corner.z = 0
                    points_list.append(corner)


        else: #LED detection
            cv_image1 = cv2.cvtColor(image_cv, cv2.COLOR_BGR2GRAY)
            cv_image1 = cv_image1[cv_image1.shape[0]/4:cv_image1.shape[0]/4*3]


    		ret,cv_image_thresh = cv2.threshold(cv_image1,220,255,cv2.THRESH_BINARY)

    		# Set up the detector with default parameters.
    		params = cv2.SimpleBlobDetector_Params()
    		params.minThreshold = 10;    # the graylevel of images
    		params.maxThreshold = 200;

    		params.filterByColor = True
    		params.blobColor = 255

    		# Filter by Area
    		params.filterByArea = False
    		params.minArea = 10000
    		params.filterByInertia = False
    		params.filterByConvexity = False
    		params.filterByCircularity = True
    		params.minCircularity = 0.1
    		detector = cv2.SimpleBlobDetector_create(params)

    		# Detect blobs.
    		keypoints = detector.detect(cv_image_thresh)
    		# Draw detected blobs as red circles.
    		# cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
    		cv_image_thresh = cv2.drawKeypoints(cv_image_thresh, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    		cv_image1 = cv2.drawKeypoints(cv_image1, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    		#print(keypoints[0].pt)
    		x1=0
    		x2=0
    		y1=0
    		y2=0
    		carfound=0
    		for key1 in keypoints:
    			for key2 in keypoints:
    				if key1!=key2:
    					if abs((key1.size-key2.size)/key1.size)<0.4: #same size keys maybe change parameter
    						if abs((key1.pt[1]-key2.pt[1]))<key1.size/1: #same y coordinate maybe change parameter
    							dist=abs((key1.pt[0]-key2.pt[0]))
    							#print(dist)
    							#print(key1.size*4+key1.size)
    							if dist>key1.size*4+key1.size and dist <key1.size*10+key1.size: #roughly right distance compared to light size
                                    points_list = []
    								#print(dist/key1.size)
                                    corner.x = key1.pt[0]
                                    corner.y = key1.pt[1]
                                    corner.z = 0
                                    points_list.append(corner)
                                    corner.x = key2.pt[0]
                                    corner.y = key2.pt[1]
                                    corner.z = 0
                                    points_list.append(corner) #pw, make this nice with loop!

    								carfound=1
                                    detection = carfound #replace carfound, but is detection actually a boolean?


        print("points list",points_list)
        vehicle_detected_msg_out.data = detection
        self.pub_detection.publish(vehicle_detected_msg_out)
        if detection:
            vehicle_corners_msg_out.header.stamp = rospy.Time.now()
            vehicle_corners_msg_out.corners = points_list
            vehicle_corners_msg_out.detection.data = detection
            vehicle_corners_msg_out.H = self.circlepattern_dims[1]
            vehicle_corners_msg_out.W = self.circlepattern_dims[0]
            self.pub_corners.publish(vehicle_corners_msg_out)
        elapsed_time = (rospy.Time.now() - start).to_sec()
        self.pub_time_elapsed.publish(elapsed_time)
        if self.publish_circles:
            cv2.drawChessboardCorners(image_cv,
                                        self.circlepattern_dims, corners, detection)
            image_msg_out = self.bridge.cv2_to_imgmsg(image_cv, "bgr8")
            self.pub_circlepattern_image.publish(image_msg_out)


if __name__ == '__main__':
    rospy.init_node('vehicle_detection', anonymous=False)
    vehicle_detection_node = VehicleDetectionNode()
    rospy.spin()
