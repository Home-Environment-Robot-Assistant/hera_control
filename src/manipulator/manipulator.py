#!/usr/bin/env python3

# Author: Lucas Lervolino Gazignato

import sys
import copy
import math

import rospy
import tf
from tf import transformations

import moveit_commander
import moveit_python

from std_msgs.msg import Empty
from geometry_msgs.msg import Pose, PoseStamped

from trajectory_msgs.msg import JointTrajectory
from moveit_msgs.msg import DisplayTrajectory

from dynamixel_workbench_msgs.srv import DynamixelCommand

from hera_control.srv import Manip3
from hera_control.srv import Manip_poses, Manip_posesResponse

from std_srvs.srv import Empty as Empty_srv


from enum import Enum

class ManipPoses(Enum):
    reset = 0
    home = 1
    open = 2
    close = 3
    pick_lixo = 4
    guardar_lixo = 5
    tirar_lixo = 6
    attack = 7
    place = 8
    point = 9
    wave = 10
    find_trash = 11
    ready_to_pick = 12

class Manipulator:

    # gripper preta:
    LEFT_GRIP_OPENED = 2341
    LEFT_GRIP_CLOSED = 1873
    LEFT_GRIP_MED = 2003

    RIGHT_GRIP_OPENED = 1805
    RIGHT_GRIP_CLOSED = 2228
    RIGHT_GRIP_MED = 2139


    # gripper branca:

    # LEFT_GRIP_OPENED = 2048
    # LEFT_GRIP_CLOSED = 1500
    # RIGHT_GRIP_OPENED = 2048
    # RIGHT_GRIP_CLOSED = 2700






    def __init__(self):
        self.group = moveit_commander.MoveGroupCommander('arm')
        self.group.set_planning_time(10)
        self.group.set_pose_reference_frame('manip_base_link')

        self.hand = moveit_commander.MoveGroupCommander('gripper')
        self.hand.set_planning_time(10)
        self.hand.set_pose_reference_frame('manip_base_link')

        self.joint_trajectory = rospy.Publisher('/dynamixel_controller/joint_trajectory', JointTrajectory, queue_size=10)
        self.update_start_state = rospy.Publisher('/rviz/moveit/update_start_state', Empty, queue_size=10)

        rospy.Subscriber('/move_group/display_planned_path', DisplayTrajectory, self.display_planned_path_callback)
        rospy.Subscriber('/dynamixel_controller/feedback', Empty, self.feedback_callback)

        self.gripper = rospy.ServiceProxy('/dynamixel_controller/dynamixel_command', DynamixelCommand)
        self.clear_octomap = rospy.ServiceProxy('/clear_octomap', Empty_srv)

        rospy.Service('manipulator', Manip3, self.handler)
        rospy.Service('manipulator_poses', Manip_poses, self.pub_poses)

        self.tf = tf.TransformListener()
        self.tf.waitForTransform('manip_base_link', 'torso', rospy.Time(), rospy.Duration(1.0))

        self.is_moving = False
        self.plan = None

        rospy.loginfo('[manip3] Going Home in 2 seconds...')
        rospy.sleep(2)
        # self.attack()
        #self.home()
        #self.open_gripper()

    def display_planned_path_callback(self, data):
        self.plan = data.trajectory[0].joint_trajectory

    def execute(self):
        self.joint_trajectory.publish(self.plan)
        self.is_moving = False

    def feedback_callback(self, data):
        self.is_moving = False
        if self.update_start_state.get_num_connections() > 0:
            self.update_start_state.publish()

    def execute_plan(self):
        self.group.plan()
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
        plan, fraction = self.group.compute_cartesian_path(waypoints, 0.001, 0, avoid_collisions = True)
        rospy.loginfo('[manip3] Cartesian path fraction: %.2f.' % fraction)
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

    def advance(self, waypoints, avoid_col = False):
        plan, fraction = self.group.compute_cartesian_path(waypoints, 0.001, 0, avoid_collisions = avoid_col)
        rospy.loginfo('[manip3] Cartesian path fraction: %.2f.' % fraction)
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

    def reset(self):
        self.group.set_named_target('reset')
        success = self.execute_plan()
        return success

    def home(self):
        self.group.set_named_target('home')
        success = self.execute_plan()
        return success

    def attack(self):
        self.group.set_named_target('attack')
        success = self.execute_plan()
        return success

    def calculo_pre_plan(self, angle):
        x = 652*(angle+1.57)+3072
        self.gripper('', 1, 'Goal_Position',int(x))

    def find_remedio(self):
        self.group.set_named_target('find_remedio')
        success = self.execute_plan()
        self.gripper('', 1, 'Goal_Position', 2078)
        self.gripper('', 2, 'Goal_Position', 2059)
        self.gripper('', 3, 'Goal_Position', 2020)
        self.gripper('', 5, 'Goal_Position', 2036)
        self.gripper('', 4, 'Goal_Position', 134)
        self.gripper('', 6, 'Goal_Position', 3400)
        return success

    def pick_remedio(self):
        self.gripper('', 4, 'Goal_Position', 300)
        rospy.sleep(0.5)
        self.gripper('', 2, 'Goal_Position', 2200)

        self.gripper('', 4, 'Goal_Position', 500)
        rospy.sleep(0.5)
        self.gripper('', 2, 'Goal_Position', 2300)

        self.gripper('', 4, 'Goal_Position', 700)
        rospy.sleep(0.5)
        self.gripper('', 2, 'Goal_Position', 2400)

        self.gripper('', 4, 'Goal_Position', 800)
        rospy.sleep(0.5)
        self.gripper('', 2, 'Goal_Position', 2500)

        self.gripper('', 4, 'Goal_Position', 900)
        rospy.sleep(0.5)
        self.gripper('', 2, 'Goal_Position', 2600)
        

        self.gripper('', 6, 'Goal_Position', 3300)

        rospy.sleep(2)
        self.close_gripper()
        rospy.sleep(2)

        self.find_remedio()
        self.attack()

        return 'success'
    
    def place_remedio(self):
        self.group.set_named_target('place_remedio')
        success = self.execute_plan()
        return success

    def find_trash(self):
        self.group.set_named_target('find_trash')
        success = self.execute_plan()
        return success

    def serve_2(self):
        self.group.set_named_target('serve_2')
        success = self.execute_plan()
        return success    

    def med_gripper(self):
        self.gripper('', 8, 'Goal_Position', Manipulator.LEFT_GRIP_MED)
        self.gripper('', 7, 'Goal_Position', Manipulator.RIGHT_GRIP_MED)
        rospy.sleep(5)
        return True
    
    def open_gripper(self):
        self.gripper('', 8, 'Goal_Position', Manipulator.LEFT_GRIP_OPENED)
        self.gripper('', 7, 'Goal_Position', Manipulator.RIGHT_GRIP_OPENED)
        rospy.sleep(5)
        return True

    def close_gripper(self):
        self.gripper('', 8, 'Goal_Position', Manipulator.LEFT_GRIP_CLOSED)
        self.gripper('', 7, 'Goal_Position', Manipulator.RIGHT_GRIP_CLOSED)
        rospy.sleep(5)
        return True

    def add_box(self,name,pose):
        scene2 = moveit_commander.PlanningSceneInterface(synchronous=True)
        pose_target = PoseStamped() # Create a new pose for the box
        pose_target.header.frame_id = "manip_base_link"
        pose_target.pose.position.x = pose.position.x + 0.1 #
        pose_target.pose.position.y = pose.position.y
        pose_target.pose.position.z = pose.position.z
        pose_target.pose.orientation = pose.orientation
    
        success = scene2.add_box(name, pose_target, size=(0.1, 0.1, 0.12))

        rospy.sleep(2)
        if success != None:
            return True
        else:
            return False

    def attach_box(self,name): 
      

        box_name = name
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        eef_link = self.group.get_end_effector_link()


       
        grasping_group = "gripper"
        touch_links = robot.get_link_names(group=grasping_group)
        success = scene.attach_box(eef_link, box_name, touch_links=touch_links)

        if success != None:
            return True
            
        else:
            return False
       
    def remove_box(self,name):

        box_name = name
  
        scene = moveit_commander.PlanningSceneInterface()
        eef_link = self.group.get_end_effector_link()

        obj = moveit_python.PlanningSceneInterface("manip_base_link")

        scene.remove_attached_object(eef_link, name=box_name)

        success = obj.removeCollisionObject(name)
        if success != None:
            return True
        else:
            return False
    
    def pick_trash(self):
        self.open_gripper()
        self.gripper('', 1, 'Goal_Position', 1973)
        self.gripper('', 2, 'Goal_Position', 2432)
        self.gripper('', 3, 'Goal_Position', 2127)
        self.gripper('', 4, 'Goal_Position', 893)
        self.gripper('', 5, 'Goal_Position', 2069)
        self.gripper('', 6, 'Goal_Position', 1945)

    def pick_lixo(self):
        self.gripper('', 1, 'Goal_Position', 2048)
        self.gripper('', 2, 'Goal_Position', 2300)
        self.gripper('', 3, 'Goal_Position', 2048)
        self.gripper('', 4, 'Goal_Position', 800)
        self.gripper('', 5, 'Goal_Position', 2048)
        self.gripper('', 6, 'Goal_Position', 2048)

        rospy.sleep(1)
        self.gripper('', 2, 'Goal_Position',  3200)
        rospy.sleep(1)
        self.gripper('', 6, 'Goal_Position', 2800)

        self.gripper('', 2, 'Goal_Position', 3400)
        rospy.sleep(1)
        self.gripper('', 4, 'Goal_Position', 1000)
    

        # x2 = 3400
        # x4 = 1000

        # for i in range (3):
        #     self.gripper(2, x2+100)
        #     rospy.sleep(1)
        #     self.gripper(4, x4+100)
        #     rospy.sleep(1)

        rospy.sleep(2)
        self.close_gripper()
        rospy.sleep(1)

        # for i in range (3):
        #     self.gripper(2, x2-100)
        #     rospy.sleep(1)
        #     self.gripper(4, x4-100)
        #     rospy.sleep(1)
        
        self.gripper('', 4, 'Goal_Position', 1000)
        rospy.sleep(1)
        self.gripper('', 2, 'Goal_Position', 3200)
        rospy.sleep(1)

        self.gripper('', 6, 'Goal_Position', 2048)
        self.gripper('', 5, 'Goal_Position', 2048)
        self.gripper('', 4, 'Goal_Position', 800)
        self.gripper('', 3, 'Goal_Position', 2048)
        self.gripper('', 2, 'Goal_Position', 2300)
        self.gripper('', 1, 'Goal_Position', 2048)

        rospy.sleep(1)
        self.home()

    def find_cima(self):
        self.attack()
        self.gripper('', 1, 'Goal_Position', 2095)
        self.gripper('', 2, 'Goal_Position', 2200)
        self.gripper('', 3, 'Goal_Position', 2023)
        self.gripper('', 4, 'Goal_Position', 1000)
        self.gripper('', 5, 'Goal_Position', 2017)
        self.gripper('', 6, 'Goal_Position', 1700)

    def drop_lixo(self):
        self.gripper('', 1, 'Goal_Position', 2048)
        self.gripper('', 2, 'Goal_Position', 2300)
        self.gripper('', 3, 'Goal_Position', 2048)
        self.gripper('', 4, 'Goal_Position', 800)
        self.gripper('', 5, 'Goal_Position', 2048)
        self.gripper('', 6, 'Goal_Position', 2048)

        rospy.sleep(1)
        self.gripper('', 2, 'Goal_Position',  3200)
        rospy.sleep(1)
        self.gripper('', 6, 'Goal_Position', 2800)

        self.gripper('', 1, 'Goal_Position', 2048)
        self.gripper('', 2, 'Goal_Position', 3000)
        self.gripper('', 3, 'Goal_Position', 2048)
        self.gripper('', 4, 'Goal_Position', 1200)
        self.gripper('', 5, 'Goal_Position', 2300)
        self.gripper('', 6, 'Goal_Position', 2200)
        self.gripper('', 2, 'Goal_Position', 3400)
        rospy.sleep(1)
        self.gripper('', 4, 'Goal_Position', 1000)
        rospy.sleep(1)
        self.gripper('', 4, 'Goal_Position', 1600)

    def guardar_lixo(self):
        self.gripper('', 6, 'Goal_Position', 2048)
        self.gripper('', 5, 'Goal_Position', 2048)
        self.gripper('', 4, 'Goal_Position', 800)
        self.gripper('', 3, 'Goal_Position', 2048)
        self.gripper('', 2, 'Goal_Position', 2300)
        self.gripper('', 1, 'Goal_Position', 2048)
        rospy.sleep(3)
        self.gripper('', 1, 'Goal_Position', 700)
        self.gripper('', 3, 'Goal_Position', 1600)
        self.gripper('', 4, 'Goal_Position', 1500)
        self.gripper('', 6, 'Goal_Position', 1500)
        rospy.sleep(4)
        
        self.gripper('', 1, 'Goal_Position', 750)
        self.gripper('', 2, 'Goal_Position', 1799)
        self.gripper('', 3, 'Goal_Position', 1771)
        self.gripper('', 5, 'Goal_Position', 1719)
        self.gripper('', 4, 'Goal_Position', 931)
        self.gripper('', 6, 'Goal_Position', 2012)
        rospy.sleep(5)
        self.open_gripper()
        self.gripper('', 6, 'Goal_Position', 2048)
        self.gripper('', 4, 'Goal_Position', 1700)
        self.gripper('', 3, 'Goal_Position', 2048)
        self.gripper('', 1, 'Goal_Position', 2048)
        rospy.sleep(5)
        self.attack()    

    def point(self,ang):
        self.gripper('', 2, 'Goal_Position', 1450)
        self.gripper('', 3, 'Goal_Position', 2048)
        self.gripper('', 5, 'Goal_Position', 2048)
        self.gripper('', 4, 'Goal_Position', 600)
        self.gripper('', 6, 'Goal_Position', 2200)
        self.gripper('', 1, 'Goal_Position', ang)
        
    def tirar_lixo(self):
        self.open_gripper()
        self.gripper('', 6, 'Goal_Position', 2048)
        self.gripper('', 5, 'Goal_Position', 2048)
        self.gripper('', 4, 'Goal_Position', 800)
        self.gripper('', 3, 'Goal_Position', 2048)
        self.gripper('', 2, 'Goal_Position', 2300)
        self.gripper('', 1, 'Goal_Position', 2048)
        rospy.sleep(4)
        self.gripper('', 1, 'Goal_Position', 500)
        self.gripper('', 4, 'Goal_Position', 1200)
        self.gripper('', 6, 'Goal_Position', 1700)
        rospy.sleep(4)   
        self.gripper('', 4, 'Goal_Position', 900)
        self.gripper('', 6, 'Goal_Position', 1900)
        rospy.sleep(2)
        self.close_gripper()
        self.gripper('', 6, 'Goal_Position', 2048)
        self.gripper('', 4, 'Goal_Position', 1500)
        self.gripper('', 1, 'Goal_Position', 2048)    

    def ready_to_pick(self):
        self.gripper('', 1, 'Goal_Position', 2048)
        self.gripper('', 2, 'Goal_Position', 2422)
        self.gripper('', 3, 'Goal_Position', 2043)
        self.gripper('', 4, 'Goal_Position', 1108)
        self.gripper('', 5, 'Goal_Position', 2047)
        self.gripper('', 6, 'Goal_Position', 2803)
        rospy.sleep(4)

    def pub_poses(self, request):
        pose_names = [n.name for n in list(ManipPoses)]
        # print(pose_names)
        return Manip_posesResponse(pose_names)

    def handler(self, request):
        type = request.type.lower()
        goal = request.goal

        print("----------------goal", goal)
        pose = Pose()
        quaternion = tf.transformations.quaternion_from_euler(goal.rx, goal.ry, goal.rz)
        pose.position.x = goal.x
        pose.position.y = goal.y
        pose.position.z = goal.z
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]

        print("----------------pose(antes)", pose)


        if type == 'reset':
            success = self.reset()
        elif type == 'home':
            success = self.home()
        elif type == 'open':
            success = self.open_gripper()
        elif type == 'med':
            success = self.med_gripper()
        elif type == 'close':
            success = self.close_gripper()
        elif type == 'pick_lixo':
            success = self.pick_lixo()
        elif type == 'pick_trash':
            success = self.pick_trash()
        elif type == 'find_trash':
            success = self.find_trash()
        elif type == 'find_remedio':
            success = self.find_remedio()
        elif type == 'pick_remedio':
            success = self.pick_remedio()
        elif type == 'place_remedio':
            success = self.place_remedio()
        elif type == 'drop_lixo':
            success = self.drop_lixo()
        elif type == 'guardar_lixo':
            success = self.guardar_lixo() 
        elif type == 'tirar_lixo':
            success = self.tirar_lixo()  
        elif type == 'ready_to_pick':
            success = self.ready_to_pick()
        elif type == 'find_cima':
            success = self.find_cima()  
        elif type == 'attack':
            success = self.attack()   
        elif type == 'pick':

            self.open_gripper() 
            # self.attack()


            ang = math.atan2(pose.position.z,pose.position.x)

            print("-------------------pose1:",pose)
            #pose.position.x -= 0.17 * math.cos(goal.rz)
            #pose.position.y -= 0.07 * math.sin(goal.rz)
            #pose.position.z += 0.1

            pose.position.x -= 0.22
            pose.position.y -= 0.08
            pose.position.z += 0.08
            
            self.calculo_pre_plan(ang)

            target_pose = copy.deepcopy(pose)           
            self.group.set_pose_target(target_pose)
            self.clear_octomap()


            success = self.execute_plan()
            rospy.sleep(5)


            #pose.position.x += 0.07 * math.cos(goal.rz)
            #pose.position.y += 0.07 * math.sin(goal.rz)
            #pose.position.z -= 0.1
            
            target_pose = copy.deepcopy(pose)
            self.group.set_pose_target(target_pose)
            self.add_box('box',pose)
            rospy.sleep(2)
            self.clear_octomap()
            self.attach_box('box')
            success = self.execute_plan()
            
            rospy.sleep(4)
            self.close_gripper() 
            rospy.sleep(1)
            self.remove_box('box')                         
            success = self.attack() 


        elif type == 'place':
            self.close_gripper() 
            pose.position.z += 0.15
            target_pose = copy.deepcopy(pose)
            self.group.set_pose_target(target_pose)
            success = self.execute_plan()
            rospy.sleep(5)
            pose.position.z -= 0.15
            target_pose = copy.deepcopy(pose)
            self.group.set_pose_target(target_pose)
            success = self.execute_plan()
            rospy.sleep(5)
            if success:
                self.open_gripper()
                target_pose = self.tf.transformPose('manip_base_link', self.group.get_current_pose()).pose
                target_pose.position.x -= 0.09 * math.cos(goal.rz)
                target_pose.position.y -= 0.09 * math.sin(goal.rz)
                self.group.set_pose_target(target_pose)
                sucess = self.execute_plan()
                rospy.sleep(5)
                if not success:
                    pose.position.z += 0.1
                    target_pose = copy.deepcopy(pose)
                    self.group.set_pose_target(target_pose)
                    sucess = self.execute_plan()
                    rospy.sleep(5)
                    self.home()
                self.home()
            if success:
                return "Success"
            else:
                return "Fail"

        elif type == 'point':
            angle = pose.position.x

            angle = (2048*angle)/1
            self.close_gripper()

            if (angle < 1024):
                angle = 1024

            elif (angle > 3072):
                angle = 3072
                        
            success = self.point(int(angle)) 
            
        elif type == 'point_people':
            pixel = pose.position.x
            angle = 2356 + (-0.96*pixel)

            self.close_gripper()
            
            if (int(angle) < 1741):
                angle = 1741

            elif (int(angle) > 2355):
                angle = 2355
                        
            success = self.point(int(angle)) 


        elif type == 'wave':
            self.close_gripper()
            angle = pose.position.x
            joint_goal = self.group.get_current_joint_values()
            # joint_goal[1] = 0.1
            # joint_goal[5] = 1
            joint_goal[4] = 0
            self.group.set_joint_value_target(joint_goal)                                                                                          
            success = self.execute_plan()
            
            # rospy.sleep(3)
            for i in range (4):
                joint_goal[4] = 0.5
                self.group.set_joint_value_target(joint_goal)
                success = self.execute_plan()

                joint_goal[4] = -0.5
                self.group.set_joint_value_target(joint_goal)
                success = self.execute_plan()
         
        else:
            self.group.set_pose_target(pose)
            success = self.execute_plan()

        if success != None:
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
