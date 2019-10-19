#!/usr/bin/env python

import cv2
import numpy as np
import os
import rospy
import yaml
import time
import csv
from duckietown import DTROS
import duckietown_utils as dtu
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from duckietown_msgs.msg import WheelsCmdStamped
from cv_bridge import CvBridge
#from ground_projection.configuration import get_extrinsics_filename

class DynamicObstacleNode(DTROS):
	"""Braitenberg Behaviour

	This node implements Braitenberg vehicle behavior on a Duckiebot.

	Args:
		node_name (:obj:`str`): a unique, descriptive name for the node
			that ROS will use

	Configuration:
		~gain (:obj:`float`): scaling factor applied to the desired
			velocity, taken from the robot-specific kinematics
			calibration
		~trim (:obj:`float`): trimming factor that is typically used
			to offset differences in the behaviour of the left and
			right motors, it is recommended to use a value that results
			in the robot moving in a straight line when forward command
			is given, taken from the robot-specific kinematics calibration
		~baseline (:obj:`float`): the distance between the two wheels
			of the robot, taken from the robot-specific kinematics
			calibration
		~radius (:obj:`float`): radius of the wheel, taken from the
			robot-specific kinematics calibration
		~k (:obj:`float`): motor constant, assumed equal for both
			motors, taken from the robot-specific kinematics calibration
		~limit (:obj:`float`): limits the final commands sent to the
			motors, taken from the robot-specific kinematics calibration

	Subscriber:
		~image/compressed (:obj:`CompressedImage`): The acquired camera
			images

	Publisher:
		~wheels_cmd (:obj:`duckietown_msgs.msg.WheelsCmdStamped`): The
			wheel commands that the motors will execute

	"""

	def __init__(self, node_name):

		# Initialize the DTROS parent class
		super(DynamicObstacleNode, self).__init__(node_name=node_name)
		self.veh_name = rospy.get_namespace().strip("/")

		# Use the kinematics calibration for the gain and trim
		self.parameters['~gain'] = None
		self.parameters['~trim'] = None
		self.parameters['~baseline'] = None
		self.parameters['~radius'] = None
		self.parameters['~k'] = None
		self.parameters['~limit'] = None

		# Set parameters using a robot-specific yaml file if such exists
		self.readParamFromFile()
		self.updateParameters()

		# Wait for the automatic gain control
		# of the camera to settle, before we stop it
		rospy.sleep(2.0)
		rospy.set_param('/%s/camera_node/exposure_mode' % self.veh_name, 'sports')
		rospy.set_param('/%s/camera_node/framerate' % self.veh_name, 10)
		self.sub = rospy.Subscriber('/%s/camera_node/image/compressed' % self.veh_name, CompressedImage, self.callback)
		self.pub = rospy.Publisher('/%s/wheels_driver_node/wheels_cmd' % self.veh_name, WheelsCmdStamped, queue_size=1)
		self.imgpub = rospy.Publisher('/%s/dynamic_obstacle/debug_img' % self.veh_name, Image, queue_size=1)
		self.imgpub2 = rospy.Publisher('/%s/dynamic_obstacle/debug_img2' % self.veh_name, Image, queue_size=1)
		self.bridge = CvBridge()
		self.mode=os.environ.get("MODE")
		self.detected_log=list()
		self.log("Initialized")

	def callback(self, image_msg):
		cv_image = self.bridge.compressed_imgmsg_to_cv2(image_msg)
		
		cv_image1 = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
		cv_image1 = cv_image1[cv_image1.shape[0]/4:cv_image1.shape[0]/4*3]



		ret,cv_image = cv2.threshold(cv_image1,220,255,cv2.THRESH_BINARY)

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
		keypoints = detector.detect(cv_image)
		# Draw detected blobs as red circles.
		# cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
		cv_image = cv2.drawKeypoints(cv_image, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
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
							
								#print(dist/key1.size)
								x1=key1.pt[0]
								x2=key2.pt[0]
								y1=key1.pt[1]
								y2=key2.pt[1]
								carfound=1

		if carfound==1:
			#fn = dtu.get_duckiefleet_root() + "/calibrations/camera_extrinsic/" + self.veh_name + ".yaml"
			f=318 #figure out how to get focal length of robot calibration

			depth=0.12*f/abs(x2-x1)
			print("Depth: " +str(depth))
			imheight, imwidth = cv_image.shape[:2]
			midt=(x1+x2)/2-imwidth/2
			Midt=midt/f*depth
			print(Midt)
			self.detected_log.append((Midt,depth))
			cv_image1=cv2.circle(cv_image1,(int(x1),int(y1)), 5, (0,255,0), 2)
			cv_image1=cv2.circle(cv_image1,(int(x2),int(y2)), 5, (0,255,0), 2)


		a=self.speedToCmd(0,0)
		msg=WheelsCmdStamped()  
		msg.vel_left=a[0]
		msg.vel_right=a[1]


		#cv_image = cv2.line(cv_image, (50,50), (100,100), (255,0,0), 9)

		msg_frame = self.bridge.cv2_to_imgmsg(cv_image,"bgr8")
		
		msg_frame2 = self.bridge.cv2_to_imgmsg(cv_image1,"bgr8")
		self.imgpub.publish(msg_frame)
		self.imgpub2.publish(msg_frame2)

		self.pub.publish(msg)


	def speedToCmd(self, speed_l, speed_r):
		"""Applies the robot-specific gain and trim to the
		output velocities

		Applies the motor constant k to convert the deisred wheel speeds
		to wheel commands. Additionally, applies the gain and trim from
		the robot-specific kinematics configuration.

		Args:
			speed_l (:obj:`float`): Desired speed for the left
				wheel (e.g between 0 and 1)
			speed_r (:obj:`float`): Desired speed for the right
				wheel (e.g between 0 and 1)

		Returns:
			The respective left and right wheel commands that need to be
				packed in a `WheelsCmdStamped` message

		"""

		# assuming same motor constants k for both motors
		k_r = self.parameters['~k']
		k_l = self.parameters['~k']

		# adjusting k by gain and trim
		k_r_inv = (self.parameters['~gain'] + self.parameters['~trim'])\
				  / k_r
		k_l_inv = (self.parameters['~gain'] - self.parameters['~trim'])\
				  / k_l

		# conversion from motor rotation rate to duty cycle
		u_r = speed_r * k_r_inv
		u_l = speed_l * k_l_inv

		# limiting output to limit, which is 1.0 for the duckiebot
		u_r_limited = self.trim(u_r,
								-self.parameters['~limit'],
								self.parameters['~limit'])
		u_l_limited = self.trim(u_l,
								-self.parameters['~limit'],
								self.parameters['~limit'])

		return u_l_limited, u_r_limited

	def readParamFromFile(self):
		"""
		Reads the saved parameters from
		`/data/config/calibrations/kinematics/DUCKIEBOTNAME.yaml` or
		uses the default values if the file doesn't exist. Adjsuts
		the ROS paramaters for the node with the new values.

		"""
		# Check file existence
		fname = self.getFilePath(self.veh_name)
		# Use the default values from the config folder if a
		# robot-specific file does not exist.
		if not os.path.isfile(fname):
			self.log("Kinematics calibration file %s does not "
					 "exist! Using the default file." % fname, type='warn')
			fname = self.getFilePath('default')

		with open(fname, 'r') as in_file:
			try:
				yaml_dict = yaml.load(in_file)
			except yaml.YAMLError as exc:
				self.log("YAML syntax error. File: %s fname. Exc: %s"
						 %(fname, exc), type='fatal')
				rospy.signal_shutdown()
				return

		# Set parameters using value in yaml file
		if yaml_dict is None:
			# Empty yaml file
			return
		for param_name in ["gain", "trim", "baseline", "k", "radius", "limit"]:
			param_value = yaml_dict.get(param_name)
			if param_name is not None:
				rospy.set_param("~"+param_name, param_value)
			else:
				# Skip if not defined, use default value instead.
				pass

	def getFilePath(self, name):
		"""
		Returns the path to the robot-specific configuration file,
		i.e. `/data/config/calibrations/kinematics/DUCKIEBOTNAME.yaml`.

		Args:
			name (:obj:`str`): the Duckiebot name

		Returns:
			:obj:`str`: the full path to the robot-specific
				calibration file

		"""
		cali_file_folder = '/data/config/calibrations/kinematics/'
		cali_file = cali_file_folder + name + ".yaml"
		return cali_file

	def trim(self, value, low, high):
		"""
		Trims a value to be between some bounds.

		Args:
			value: the value to be trimmed
			low: the minimum bound
			high: the maximum bound

		Returns:
			the trimmed value
		"""

		return max(min(value, high), low)

	def onShutdown(self):
		"""Shutdown procedure.

		Publishes a zero velocity command at shutdown."""
		#self.driver.setWheelsSpeed(left=0.0, right=0.0)
		# MAKE SURE THAT THE LAST WHEEL COMMAND YOU PUBLISH IS ZERO,
		# OTHERWISE YOUR DUCKIEBOT WILL CONTINUE MOVING AFTER
		# THE NODE IS STOPPED

		# PUT YOUR CODE HERE
		with open('/data/detected_duckiebot_log.csv', mode='w') as log:
			log = csv.writer(log, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
			for data in self.detected_log:
				log.writerow([data[0], data[1]])



		msg=WheelsCmdStamped()
		msg.vel_left=0
		msg.vel_right=0
		self.pub.publish(msg)
		time.sleep(0.5)
		self.pub.publish(msg)
		print("zero wheel coomand sent")


		super(DynamicObstacleNode, self).onShutdown()


if __name__ == '__main__':
	# Initialize the node
	camera_node = DynamicObstacleNode(node_name='dynamicObstacle')
	# Keep it spinning to keep the node alive
	rospy.spin()
	#self.driver.setWheelsSpeed(left=0.5, right=0.5)
	#time.sleep(1)
	#self.driver.setWheelsSpeed(left=0.0, right=0.0)
