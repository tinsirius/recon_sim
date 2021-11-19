#!/usr/bin/env python

import rospy, tf, rospkg, random
from gazebo_msgs.srv import DeleteModel, SpawnModel, GetModelState
from geometry_msgs.msg import Quaternion, Pose, Point

class CubeSpawner():

	def __init__(self):
		self.rospack = rospkg.RosPack()
		self.path = self.rospack.get_path('recon_sim')+"/urdf/"
		self.cubes = []
		self.cubes.append(self.path+"aruco0.urdf")
		self.col = 0

		self.sm = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)
		self.dm = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
		self.ms = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

	def checkModel(self):
		res = self.ms("cube", "world")
		return res.success

	def getPosition(self):
		res = self.ms("cube", "world")
		return res.pose.position.z

	def spawnModel(self):
		# print(self.col)
		cube = self.cubes[self.col]
		with open(cube,"r") as f:
			cube_urdf = f.read()
		
		quat = tf.transformations.quaternion_from_euler(0,0,0)
		orient = Quaternion(quat[0],quat[1],quat[2],quat[3])
		pose = Pose(Point(x=0,y=-0.55,z=0.75), orient)
		self.sm("cube", cube_urdf, '', pose, 'world')
		rospy.sleep(1)

	def deleteModel(self):
		self.dm("cube")
		rospy.sleep(1)

	def shutdown_hook(self):
		self.deleteModel()
		print("Shutting down")


if __name__ == "__main__":
	print("Waiting for gazebo services...")
	rospy.init_node("spawn_cubes")
	rospy.wait_for_service("/gazebo/delete_model")
	rospy.wait_for_service("/gazebo/spawn_urdf_model")
	rospy.wait_for_service("/gazebo/get_model_state")
	r = rospy.Rate(15)
	cs = CubeSpawner()
	rospy.on_shutdown(cs.shutdown_hook)
	cs.spawnModel()
	while not rospy.is_shutdown():		
		if cs.getPosition()<0.05:
			cs.deleteModel()
			cs.spawnModel()
		r.sleep()
		
