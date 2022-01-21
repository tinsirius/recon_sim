#!/usr/bin/env python2

import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rospy
from geometry_msgs.msg import WrenchStamped, PoseArray, Pose, PoseStamped
from actionlib_msgs.msg import GoalStatusArray
import geometry_msgs.msg
from panda_utils_python.panda_utils import panda_utils
import math
from scipy.spatial.transform import Rotation as R
import numpy as np
import copy
import actionlib
from franka_msgs.msg import FrankaState
import recon_controllers.msg
import time
import argparse

class JointPoses:
    READY_CLASSIC = [0, -math.pi/4, 0, -3 * math.pi / 4, 0, math.pi / 2, math.pi / 4]
    DROP = [math.pi/2, -math.pi/4, 0, -3 * math.pi / 4, 0, math.pi / 2, math.pi / 4]
    READY = [0, 0, 0, -math.pi / 2, 0, math.pi / 2, math.pi / 4]

class VisualServo: 

	def __init__(self, ns=""):
		self.ns = ns
		self.cv_bridge = CvBridge()
		self.cx = None
		self.cy = None
		self.targeted_area = 70000
		rospy.Subscriber(self.ns + "/camera/color/image_raw", Image, self.ImageCallback, queue_size=1)
		self.Gain = {"x":-0.2, "y":-0.2, "z":-0.0001}
		self.pub = rospy.Publisher(self.ns + '/force_servo_controller/target_wrench', WrenchStamped, queue_size=10)
		self.seeing_marker = False
		self.count = 0
		self.helper = panda_utils(self.ns)

	def move_joint_pose(self, pose, duration = 5.0):
		# client = actionlib.SimpleActionClient(self.ns + '/set_target_joint', recon_controllers.msg.set_target_jointAction)
		client = actionlib.SimpleActionClient(self.ns + '/joint_position_controller/set_target_joint', recon_controllers.msg.set_target_jointAction)
		client.wait_for_server()
		goal = recon_controllers.msg.set_target_jointGoal()
		goal.joints = pose
		goal.duration = duration
		client.send_goal(goal)
		is_success = client.wait_for_result(timeout = rospy.Duration(duration + 5))
		if not is_success:
			return False
		result = client.get_result()
		return result.success

	def move_delta_cartesian(self, delta_pose):
		current_state = rospy.wait_for_message(self.ns + "/franka_state_controller/franka_states", FrankaState)
		current_transformation = np.array(current_state.O_T_EE).reshape((4, 4)).transpose()
		current_orientation = current_transformation[0:3, 0:3]
		current_translation = current_transformation[0:3, 3]
		delta_angle = R.from_euler('xyz', delta_pose[3:], degrees=True)
		target_angle = R.from_dcm(np.matmul(current_orientation, R.as_dcm(delta_angle))).as_quat()
		pub = rospy.Publisher(self.ns + '/cartesian_pose_controller/equilibrium_pose', PoseStamped, queue_size=1)
		msg = PoseStamped()
		msg.pose.position.x = current_translation[0] + delta_pose[0]
		msg.pose.position.y = current_translation[1] + delta_pose[1]
		msg.pose.position.z = current_translation[2] + delta_pose[2]
		msg.pose.orientation.x = target_angle[0]
		msg.pose.orientation.y = target_angle[1]
		msg.pose.orientation.z = target_angle[2]
		msg.pose.orientation.w = target_angle[3]
		pub_rate = rospy.Rate(10)
		for i in range(10):
			pub.publish(msg)
			pub_rate.sleep()

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
				cv2.circle(original, (cX, cY), 4, (0, 0, 255), -1)
				cv2.circle(original, (cx, cy), 4, (0, 0, 255), -1)
				cv2.line(original, (cX, cY), (cx, cy), (255, 0, 0), 1)
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
			self.helper.switch_controller('joint_position_controller')
			self.move_joint_pose(JointPoses.READY)
			
			while not self.seeing_marker:
				rate.sleep()

			self.helper.switch_controller('force_servo_controller')

			# ----------REPLACE THIS WITH YOUR STOPPING CONDITION------------
			time.sleep(3)
			# ---------------------------------------------------------------

			self.helper.switch_controller('cartesian_pose_controller')
			# self.move_delta_cartesian([0.03, 0.2, -0.06, 0, 0, -90])
			self.move_delta_cartesian([0.03, 0.2, -0.06, 0, 0, -90])

			# ----------REPLACE THIS WITH YOUR PREDICTION------------
			time.sleep(3)
			# -------------------------------------------------------

			self.helper.close_gripper(0.05, 10, 0.5)
			self.helper.switch_controller('joint_position_controller')
			self.move_joint_pose(JointPoses.READY_CLASSIC)
			self.move_joint_pose(JointPoses.DROP)
			self.helper.move_gripper(width = 0.08)
			self.move_joint_pose(JointPoses.READY)

			rate.sleep()



if __name__ == '__main__':
	parser = argparse.ArgumentParser(description='Control Script for ReCon Project')
	parser.add_argument('--armID', type=str, help='arm ID of the robot', default="")
	args = parser.parse_args()

	rospy.init_node("visual_servo_" + args.armID)
	visual_servo = VisualServo(args.armID)
	visual_servo.helper.move_gripper(width = 0.08)
	visual_servo.main_loop()
	rospy.spin()
