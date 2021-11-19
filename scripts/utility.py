#!/usr/bin/env python

import rospy
from actionlib_msgs.msg import GoalStatusArray
from controller_manager_msgs.srv import *
import geometry_msgs.msg
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
import math
import numpy as np
import copy
import actionlib
from franka_msgs.msg import FrankaState
import franka_gripper.msg
import recon_controllers.msg
import time
from scipy.spatial.transform import Rotation as R

class JointPoses:
    READY_CLASSIC = [0, -math.pi/4, 0, -3 * math.pi / 4, 0, math.pi / 2, math.pi / 4]
    DROP = [math.pi/2, -math.pi/4, 0, -3 * math.pi / 4, 0, math.pi / 2, math.pi / 4]
    READY = [0, 0, 0, -math.pi / 2, 0, math.pi / 2, math.pi / 4]

def move_joint_pose(pose, duration = 5.0):
    client = actionlib.SimpleActionClient('/set_target_joint', recon_controllers.msg.set_target_jointAction)
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

def move_delta_cartesian(delta_pose):
    current_state = rospy.wait_for_message("/panda/franka_state_controller/franka_states", FrankaState)
    current_transformation = np.array(current_state.O_T_EE).reshape((4, 4)).transpose()
    current_orientation = current_transformation[0:3, 0:3]
    current_translation = current_transformation[0:3, 3]
    delta_angle = R.from_euler('xyz', delta_pose[3:], degrees=True)
    target_angle = R.from_dcm(np.matmul(current_orientation, R.as_dcm(delta_angle))).as_quat()
    pub = rospy.Publisher('/panda/cartesian_pose_controller/equilibrium_pose', PoseStamped, queue_size=1)
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



def close_gripper(width, force, speed, epsilon_inner = 0.1, epsilon_outer = 0.1):
    client = actionlib.SimpleActionClient('/panda/franka_gripper/grasp', franka_gripper.msg.GraspAction)
    client.wait_for_server()
    goal = franka_gripper.msg.GraspGoal()
    goal.width = width
    goal.epsilon.inner = epsilon_inner
    goal.epsilon.outer = epsilon_outer
    goal.speed = speed
    goal.force = force
    client.send_goal(goal)
    is_success = client.wait_for_result(timeout = rospy.Duration(10))
    if not is_success:
        return False
    result = client.get_result()
    return result.success

def move_gripper(width = 0.07, speed = 0.5):
    client = actionlib.SimpleActionClient('/panda/franka_gripper/move', franka_gripper.msg.MoveAction)
    client.wait_for_server()
    goal = franka_gripper.msg.MoveGoal()
    goal.width = width
    goal.speed = speed
    client.send_goal(goal)
    is_success = client.wait_for_result(timeout = rospy.Duration(10))
    if not is_success:
        return False
    result = client.get_result()
    return result.success

def stop_gripper():
    client = actionlib.SimpleActionClient('/panda/franka_gripper/stop', franka_gripper.msg.StopAction)
    client.wait_for_server()
    goal = franka_gripper.msg.StopGoal()
    client.send_goal(goal)
    is_success = client.wait_for_result(timeout = rospy.Duration(10))
    if not is_success:
        return False
    result = client.get_result()
    return result.success

def home_gripper():
    client = actionlib.SimpleActionClient('/panda/franka_gripper/homing', franka_gripper.msg.HomingAction)
    client.wait_for_server()
    goal = franka_gripper.msg.HomingGoal()
    client.send_goal(goal)
    is_success = client.wait_for_result(timeout = rospy.Duration(10))
    if not is_success:
        return False
    result = client.get_result()
    return result.success

def switch_controller(controller_name):
    rospy.wait_for_service('/panda/controller_manager/list_controllers')
    try:
        list_client = rospy.ServiceProxy('/panda/controller_manager/list_controllers', ListControllers)
        list_object = ListControllersRequest()
        controllers = list_client(list_object)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
    
    is_loaded = False
    for controller in controllers.controller:
        if controller_name == controller.name:
            is_loaded = True
    
    if is_loaded:
        active_controller = ''
        for controller in controllers.controller:
            if controller.state == 'running':
                if 'state' not in controller.name:
                    active_controller = controller.name
        if active_controller == controller_name:
            print("robot is already running " + controller_name)
            return True
        else:
            rospy.wait_for_service('/panda/controller_manager/switch_controller')
            try:
                switch_client = rospy.ServiceProxy('/panda/controller_manager/switch_controller', SwitchController)
                switch_object = SwitchControllerRequest()
                switch_object.stop_controllers = [active_controller]
                switch_object.start_controllers = [controller_name]
                switch_object.strictness = 2
                switch_object.start_asap = False
                switch_object.timeout = 0.0
                result = switch_client(switch_object)
                if result.ok:
                    print("Switch to " + controller_name)
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
            return True
    else:
        print("Controller is not loaded yet")
        return False
