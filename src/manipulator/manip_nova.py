#!/usr/bin/env python3

import sys
import copy
import math

import rospy
import tf
from tf import transformations

import moveit_commander

from std_msgs.msg import Empty
from geometry_msgs.msg import Pose

from trajectory_msgs.msg import JointTrajectory
from moveit_msgs.msg import DisplayTrajectory

from dynamixel_workbench_msgs.srv import DynamixelCommand

from manip3.srv import Manip3

class Manipulator:
    def __init__(self):
        #iniciando o moveit
        self.arm = moveit_commander.MoveGroupCommander('arm') 
        self.hand = moveit_commander.MoveGroupCommander('gripper')
      
        #referencia para integracao com a visao
        self.arm.set_pose_reference_frame('manip_base_link')
        self.hand.set_pose_reference_frame('manip_base_link')

        self.__hand_traj_client = SimpleActionClient("/gripper_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
        self.__arm_traj_client = SimpleActionClient("/arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction)

        if self.__hand_traj_client.wait_for_server(timeout=rospy.Duration(60.0)) is False:
            rospy.logfatal("Failed to connect to /gripper_controller/follow_joint_trajectory in 4sec.")
            raise Exception("Failed to connect to /gripper_controller/follow_joint_trajectory in 4sec.")
                                              
        if self.__arm_traj_client.wait_for_server(timeout=rospy.Duration(60.0)) is False:
            rospy.logfatal("Failed to connect to /arm_controller/follow_joint_trajectory in 4sec.")
            raise Exception("Failed to connect to /arm_controller/follow_joint_trajectory in 4sec.")


        self.joint_trajectory = rospy.Publisher('/joint_trajectory', JointTrajectory, queue_size=10) 
        self.update_start_state = rospy.Publisher('/rviz/moveit/update_start_state', Empty, queue_size=10)

        # planejamento do moveit
        rospy.wait_for_service('/get_planning_scene', 60.0) 
        self.__get_planning_scene = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
        self.__pub_planning_scene = rospy.Publisher('/planning_scene', PlanningScene, queue_size=10, latch=True)

        # criando service para passar posições para o robo 
        rospy.Service('manipulator', setGoal, self.handler) 

        self.tf = tf.TransformListener() #
        self.tf.waitForTransform('manip_base_link', 'base', rospy.Time(), rospy.Duration(1.0))                                       
