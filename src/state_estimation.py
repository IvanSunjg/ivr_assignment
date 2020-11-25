#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
import message_filters
from std_msgs.msg import Float64MultiArray, Int16MultiArray, Float64

class joint_state:
    def __init__(self):
        # initialize the node named joint_processing
        rospy.init_node('joint_processing', anonymous=True)

        # initialize subscribers to receive the coordinates of the joints
        self.robot_x_z = message_filters.Subscriber("/robot/x_z", Int16MultiArray)
        self.robot_y_z = message_filters.Subscriber("/robot/y_z", Int16MultiArray)
        self.sync = message_filters.ApproximateTimeSynchronizer([self.robot_x_z, self.robot_y_z],
                                                                 10, 0.1,allow_headerless=True)

        self.sync.registerCallback(self.callback)
        # initialize a publisher for joint angles
        self.joints_pub = rospy.Publisher("/joints_ang", Float64MultiArray, queue_size=10)
        # initialize a publisher for target object position
        self.target_pub = rospy.Publisher("/target_pos", Float64MultiArray, queue_size=10)
        # initialize a publisher for end effector position
        self.end_eff_pub = rospy.Publisher("/end_pos", Float64MultiArray, queue_size=10)

        self.joint2_pub = rospy.Publisher("/joints_ang2", Float64, queue_size=10)
        self.joint3_pub = rospy.Publisher("/joints_ang3", Float64, queue_size=10)
        self.joint4_pub = rospy.Publisher("/joints_ang4", Float64, queue_size=10)

        rospy.sleep(0.4)

        #[red_x,red_z,green_x,green_z,blue_x,blue_z,target_x,target_z]
        self.x_z = None
        # [red_y,red_z,green_y,green_z,blue_y,blue_z,target_y,target_z]
        self.y_z = None

    def detect_joint_angles(self,red,green,blue):
        new_z_1 = np.sqrt((green[0]-blue[0])**2+(green[2]-blue[2])**2)
        joint_2 = np.arctan2(np.abs(green[1]-blue[1]),new_z_1)
        if(green[1]>blue[1]):
            joint_2 = -joint_2
        new_z_2 = np.sqrt((green[1]-blue[1])**2+(green[2]-blue[2])**2)
        joint_3 = np.arctan2(np.abs(green[0]-blue[0]),new_z_2)
        if (green[0]<blue[0]):
            joint_3 = -joint_3
        blue_to_green = green - blue
        green_to_red = red - green
        joint_4 = np.arccos(np.dot(blue_to_green, green_to_red) /
                                    (np.linalg.norm(blue_to_green) * np.linalg.norm(green_to_red)))
        s_1 = np.sqrt((green[0]-blue[0])**2+(green[1]-blue[1])**2)
        s_2 = np.sqrt((red[0]-blue[0])**2+(red[1]-blue[1])**2)
        if(joint_2<0):
            if((red[2]-blue[2])<(s_2/s_1)*(green[2]-blue[2])):
                joint_4 = -joint_4
        if (joint_2>0):
            if((red[2]-blue[2])>(s_2/s_1)*(green[2]-blue[2])):
                joint_4 = -joint_4
        return np.array([joint_2,joint_3,joint_4])

    def callback(self,x_z_pos,y_z_pos):
        # xz [red_x,red_z,green_x,green_z,blue_x,blue_z,target_x,target_z]
        # yz [red_y,red_z,green_y,green_z,blue_y,blue_z,target_y,target_z]
        self.x_z = x_z_pos.data
        self.y_z = y_z_pos.data
        self.red_x = self.x_z[0]
        self.red_y = self.y_z[0]
        self.red_z_1 = self.x_z[1]
        self.red_z_2 = self.y_z[1]
        self.green_x = self.x_z[2]
        self.green_y = self.y_z[2]
        self.green_z_1 = self.x_z[3]
        self.green_z_2 = self.y_z[3]
        self.blue_x = self.x_z[4]
        self.blue_y = self.y_z[4]
        self.blue_z_1 = self.x_z[5]
        self.blue_z_2 = self.y_z[5]
        self.target_x = self.x_z[6]
        self.target_y = self.y_z[6]
        self.target_z_1 = self.x_z[7]
        self.target_z_2 = self.y_z[7]
        if(self.red_z_2 == 0):self.red_z = self.red_z_1
        else: self.red_z = self.red_z_2
        if (self.green_z_2 == 0):self.green_z = self.green_z_1
        else:self.green_z = self.green_z_2
        if (self.blue_z_2 == 0):self.blue_z = self.blue_z_1
        else:self.blue_z = self.blue_z_2
        if (self.target_z_2 == 0): self.target_z =self.target_z_1
        else:self.target_z = self.target_z_2
        self.red = np.array([self.red_x-392,self.red_y-392,529-self.red_z])
        self.green = np.array([self.green_x-392,self.green_y-392,529-self.green_z])
        self.blue = np.array([self.blue_x-392,self.blue_y-392,529-self.blue_z])
        self.target = np.array([(self.target_x-392)*0.04,
                                (self.target_y-392)*0.04,(529-self.target_z)*0.04])
        self.joint_angles = Float64MultiArray()
        self.joint_angles.data = self.detect_joint_angles(self.red,self.green,self.blue)
        joint2 = self.joint_angles.data[0]
        joint3 = self.joint_angles.data[1]
        joint4 = self.joint_angles.data[2]
        self.red = np.array([(self.red_x - 392)*0.04,(self.red_y - 392)*0.04,(529 - self.red_z)*0.04])
        self.target_pos = Float64MultiArray()
        self.target_pos.data = self.target
        self.end_eff_pos = Float64MultiArray()
        self.end_eff_pos.data = self.red
        # Publish the joint angles
        self.joints_pub.publish(self.joint_angles)
        self.target_pub.publish(self.target_pos)
        self.end_eff_pub.publish(self.end_eff_pos)

        self.joint2_pub.publish(joint2)
        self.joint3_pub.publish(joint3)
        self.joint4_pub.publish(joint4)


# call the class
def main(args):
  ic = joint_state()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)
