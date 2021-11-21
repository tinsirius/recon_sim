#!/usr/bin/env python2

import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rospy
from geometry_msgs.msg import WrenchStamped
from utility import *

class VisualServo: 

	def __init__(self):
		self.cv_bridge = CvBridge()
		self.cx = None
		self.cy = None
		self.targeted_area = 70000
		rospy.Subscriber("/camera/color/image_raw", Image, self.ImageCallback, queue_size=1)
		self.Gain = {"x":-0.2, "y":-0.2, "z":-0.0001}
		self.pub = rospy.Publisher('/panda/force_servo_controller/target_wrench', WrenchStamped, queue_size=10)
		self.seeing_marker = False
		self.count = 0

	def ImageCallback(self, msg):
		original = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
		height, width, channels = original.shape
		grayImage = cv2.cvtColor(original, cv2.COLOR_BGR2GRAY)
		(thresh, bw) = cv2.threshold(grayImage, 20, 255, cv2.THRESH_BINARY)
		arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
		arucoParams = cv2.aruco.DetectorParameters_create()
		(corners, ids, rejected) = cv2.aruco.detectMarkers(bw, arucoDict, parameters=arucoParams)

		if len(corners) > 0:
			self.seeing_marker = True
			self.count = 0
			ids = ids.flatten()
			for (markerCorner, markerID) in zip(corners, ids):
				corners = markerCorner.reshape((4, 2))
				(topLeft, topRight, bottomRight, bottomLeft) = corners

				topRight = (int(topRight[0]), int(topRight[1]))
				bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
				bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
				topLeft = (int(topLeft[0]), int(topLeft[1]))

				cX = int((topLeft[0] + bottomRight[0]) / 2.0)
				cY = int((topLeft[1] + bottomRight[1]) / 2.0)
				cx = width/2
				cy = height/2
				cv2.circle(original, (int(cX), int(cY)), 4, (0, 0, 255), -1)
				cv2.circle(original, (int(cx), int(cy)), 4, (0, 0, 255), -1)
				cv2.line(original, (int(cX), int(cY)), (int(cx), int(cy)), (255, 0, 0), 1)
				area = self.polygon_area(topLeft, topRight, bottomRight, bottomLeft)

				wrench = WrenchStamped()
				wrench.wrench.force.y = self.Gain["x"] * (cX - cx)
				wrench.wrench.force.x = self.Gain["y"] * (cY - cy)
				wrench.wrench.force.z = self.Gain["z"] * (self.targeted_area - area)
				self.pub.publish(wrench)
		else:
			self.seeing_marker = False
			self.count = self.count + 1

		cv2.imshow("image", original)
		cv2.waitKey(1)

	def polygon_area(self, vertice1, vertice2, vertice3, vertice4):
		return (1.0/2) * (vertice1[0] * vertice2[1]
							+ vertice2[0] * vertice3[1]
							+ vertice3[0] * vertice4[1]
							+ vertice4[0] * vertice1[1]
							- vertice2[0] * vertice1[1]
							- vertice3[0] * vertice2[1]
							- vertice4[0] * vertice3[1]
							- vertice1[0] * vertice4[1])
	
	def main_loop(self):
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			switch_controller('joint_position_controller')
			move_joint_pose(JointPoses.READY)
			
			while not self.seeing_marker:
				rate.sleep()

			switch_controller('force_servo_controller')

			# ----------REPLACE THIS WITH YOUR STOPPING CONDITION------------
			time.sleep(3)
			# ---------------------------------------------------------------

			switch_controller('cartesian_pose_controller')
			move_delta_cartesian([0.03, 0.2, -0.06, 0, 0, -90])

			# ----------REPLACE THIS WITH YOUR PREDICTION------------
			time.sleep(3)
			# -------------------------------------------------------

			close_gripper(0.05, 10, 0.5)
			switch_controller('joint_position_controller')
			move_joint_pose(JointPoses.READY_CLASSIC)
			move_joint_pose(JointPoses.DROP)
			move_gripper(width = 0.08)
			move_joint_pose(JointPoses.READY)

			rate.sleep()



if __name__ == '__main__':
	rospy.init_node("visual_servo")
	move_gripper(width = 0.08)
	visual_servo = VisualServo()
	visual_servo.main_loop()
	rospy.spin()
