#!/usr/bin/env python

from gazebo_msgs.srv import SetModelConfiguration
from std_msgs.msg import Float64
import rospy

class HeadControl():
    """docstring for HeadControl."""

    def __init__(self):

        self.rate = rospy.Rate(10)

        self.model_name = "robot"
        self.urdf = "robot"
        self.joint = "joint_torso_to_head"
        self.head_ang = 1.5707


        rospy.Subscriber('/head_control', Float64, self.callback)


        rospy.loginfo('Waiting for "/gazebo/set_model_state" service')
        rospy.wait_for_service('/gazebo/set_model_state')
        self.model_config = rospy.ServiceProxy('/gazebo/set_model_state', SetModelConfiguration)

        rospy.loginfo('Head_control is ready!')

        while not rospy.is_shutdown():
            self.rate.sleep()

            # modelconfigMsg = SetModelConfigurationRequest()
            # modelconfigMsg.model_name = model_name
            # modelconfigMsg.urdf_param_name = urdf
            # modelconfigMsg.joint_names.append(joint)
            # modelconfigMsg.joint_positions.append(head_ang)
            self.model_config(self.model_name, self.urdf, self.joint, self.head_ang)

    def callback(self, msg):
        self.head_ang = msg.data


if __name__ == "__main__":
    rospy.init_node('head_control')
    hc = HeadControl()
