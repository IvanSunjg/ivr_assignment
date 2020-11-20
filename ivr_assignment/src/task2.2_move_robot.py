#!/usr/bin/env python3

import roslib
import sys
import rospy
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray, Float64

class robot_move:

    def __init__(self):
        # initialize the node named image_processing
        rospy.init_node('robot_move', anonymous=True)
        # publish angle for joints 2,3,4
        self.robot_joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
        self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
        self.robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)
        # run the call back
        self.end_effector_sub2 = rospy.Subscriber("/end_pos",Float64MultiArray,self.callback)

    def call(self,data):
        # set the angles 
        self.joint2=Float64()
        self.joint2.data = np.pi/2*np.sin(np.pi/15*rospy.get_time())
        self.joint3=Float64()
        self.joint3.data = np.pi/2*np.sin(np.pi/18*rospy.get_time())
        self.joint4=Float64()
        self.joint4.data = np.pi/2*np.sin(np.pi/20*rospy.get_time())
        # publish angles
        self.robot_joint2_pub.publish(self.joint2)
        self.robot_joint3_pub.publish(self.joint3)
        self.robot_joint4_pub.publish(self.joint4)
        rate = rospy.Rate(50)
        rate.sleep()
