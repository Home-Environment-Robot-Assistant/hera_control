
#!/usr/bin/env python3

from pickle import TRUE
import sys
import copy
import math

import rospy
import tf

import moveit_commander 
from moveit_msgs.msg import PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene

from actionlib import SimpleActionClient
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

from std_msgs.msg import Empty
from geometry_msgs.msg import Pose

from trajectory_msgs.msg import JointTrajectory
from moveit_msgs.msg import DisplayTrajectory

from hera_manipulation.srv import setGoal

class Manipulator:

    def __init__(self):
                
        self.arm = moveit_commander.MoveGroupCommander('arm')
        self.hand = moveit_commander.MoveGroupCommander('gripper')

        # self.group.set_planning_time(10)
        self.arm.set_pose_reference_frame('manip_base_link')
        # self.hand.set_planning_time(10)
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

        rospy.wait_for_service('/get_planning_scene', 60.0)
        self.__get_planning_scene = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
        self.__pub_planning_scene = rospy.Publisher('/planning_scene', PlanningScene, queue_size=10, latch=True)

        rospy.Service('manipulator', setGoal, self.handler)

        self.tf = tf.TransformListener()
        self.tf.waitForTransform('manip_base_link', 'base', rospy.Time(), rospy.Duration(1.0))                                       
                                              

        self.is_moving = False
        

        rospy.loginfo('[manip] Going Home in 5 seconds...')
        self.home()

    def display_planned_path_callback(self, data):
        self.plan = data.trajectory[0].joint_trajectory

    def execute(self):
        self.joint_trajectory.publish(self.plan)
        self.is_moving = True
        

    def feedback_callback(self, data):
        self.is_moving = False
        if self.update_start_state.get_num_connections() > 0:
            self.update_start_state.publish()

    def execute_plan(self):
        self.arm.plan()
        rospy.sleep(2)
        if self.plan != None:
            self.execute()
            while self.is_moving:
                pass
            rospy.sleep(2)
            self.plan = None
            return True
        else:
            return False

    def execute_cartesian_plan(self, waypoints):
        plan, fraction = self.arm.compute_cartesian_path(waypoints, 0.001, 0, avoid_collisions = True)
        rospy.loginfo('[manip] Cartesian path fraction: %.2f.' % fraction)
        rospy.sleep(2)
        if self.plan != None and fraction > 0.9:
            self.arm.execute(plan, wait=True)
           
            rospy.sleep(5)
            self.plan = None
            return True
        else:
            self.plan = None
            return False

    def advance(self, waypoints, avoid_col = False):
        plan, fraction = self.arm.compute_cartesian_path(waypoints, 0.001, 0, avoid_collisions = avoid_col)
        rospy.loginfo('[manip] Cartesian path fraction: %.2f.' % fraction)
        rospy.sleep(2)
        if self.plan != None and fraction > 0.9:
            self.execute()
            while self.is_moving:
                pass
            rospy.sleep(5)
            self.plan = None
            return True
        else:
            self.plan = None
            return False

    def reset_manipulator(self):
        self.arm.set_named_target('home')
        plan = self.arm.plan()
        if not self.arm.execute(plan, wait=True):
            return False
        self.open_gripper()
        return True

    def home(self):
        self.arm.set_named_target('home')
        plan = self.arm.plan()
        if not self.arm.execute(plan, wait=True):
            return False
        return True
    
    def front(self):
        self.arm.set_named_target('front')
        plan = self.arm.plan()
        if not self.arm.execute(plan, wait=True):
            return False
        return True
    
    def attack(self):
        self.arm.set_named_target('attack')
        plan = self.arm.plan()
        if not self.arm.execute(plan, wait=True):
            return False
        return True

    def open_gripper(self):
        self.hand.set_named_target('open_gripper')
        plan = self.hand.plan()
        if not self.hand.execute(plan, wait=True):
            return False
        return True

    def close_gripper(self):
        self.hand.set_named_target('close_gripper')
        plan = self.hand.plan()
        success = self.hand.execute(plan, wait=True)
        return success

    def handler(self, request):
        type = request.type.lower()
        goal = request.goal

        pose = Pose()
        quaternion = tf.transformations.quaternion_from_euler(goal.rx, goal.ry, goal.rz)
        pose.position.x = goal.x
        pose.position.y = goal.y
        pose.position.z = goal.z
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]

        if type == 'reset':
            success = self.reset_manipulator()
        elif type == 'home':
            success = self.home()
        elif type == 'open':
            success = self.open_gripper()
        elif type == 'old_close':
            success = self.close_gripper()
        elif type == 'front':
            success = self.front()
        elif type == 'attack':
            success = self.attack()
        elif type == '':
            
            target_pose = copy.copy(pose)
            self.arm.set_pose_target(target_pose)
            plan = self.arm.plan()
            success = self.arm.execute(plan, wait=True)
            if success:
                return 'SUCCEEDED'
            else:
                return 'FAILED'
               
        elif type == 'pick':
            
            target_pose = copy.deepcopy(pose)
            self.arm.set_pose_target(target_pose)
            plan = self.arm.plan()
            success = self.arm.execute(plan, wait=True)             
           
            rospy.sleep(10)
            self.close_gripper()
            rospy.sleep(5)
            self.home() 

            if success:
                return 'SUCCEEDED'
            else:
                return 'FAILED'
            
        elif type == 'place':
            target_pose = copy.deepcopy(pose)
            self.arm.set_pose_target(target_pose)
            plan = self.arm.plan()
            success = self.arm.execute(plan, wait=True)

            rospy.sleep(5)
            self.open_gripper()                            
            rospy.sleep(10)
            self.home() 
       
        elif type == 'point':
            self.close_gripper()
            joint_goal = self.arm.get_current_joint_values()
            joint_goal[0] = 0.35
            joint_goal[1] = 0.0
            joint_goal[2] = 0.0
            joint_goal[3] = -2.35
            joint_goal[4] = 0.0
            joint_goal[5] = -0.35
            self.arm.set_joint_value_target(joint_goal)
            plan = self.arm.plan()
            success = self.arm.execute(plan, wait=True)
            
            if success:
                return 'SUCCEEDED'
            else:
                return 'FAILED'

if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('manipulator', log_level=rospy.INFO)
    Manipulator()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass