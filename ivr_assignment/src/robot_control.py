#!/usr/bin/env python3

import roslib
import sys
import rospy
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray, Float64



class robot_control:

  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('robot_control', anonymous=True)

    # subscriber for joint angles
    self.joints_sub = rospy.Subscriber("/joints_ang",Float64MultiArray,self.callback)
    # subscriber for target object position
    self.target_sub = rospy.Subscriber("/target_pos",Float64MultiArray,self.callback)
    # subscriber for end_effector position
    self.end_effector_sub = rospy.Subscriber("/end_effector_pos",Float64MultiArray,self.callback)
    
    # initialize a publisher to send predicted robot end-effector position
    self.end_effector_pub = rospy.Publisher("/end_effector_pred",Float64MultiArray, queue_size=10)
    
    # initialize a publisher to send joints' angular position to the robot
    self.robot_joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
    self.robot_joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
    self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
    self.robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)

    # initialize errors
    self.time_previous_step = np.array([rospy.get_time()], dtype='float64') 

    # initialize error and derivative of error for trajectory tracking  
    self.error = np.array([0.0,0.0,0.0], dtype='float64')  
    self.error_d = np.array([0.0,0.0,0.0], dtype='float64') 

  # Calculate the forward kinematics
  def forward_kinematics(self,joints):
    a,b,c,d = joints
    x = np.sin(a)*(np.sin(b)*np.cos(c)*(3*np.cos(d)+3.5)+3*np.cos(b)*np.sin(d)) + np.cos(a)*np.sin(c)*(3*np.cos(d)+3.5)
    y = np.cos(a)*(np.sin(b)*np.cos(c)*(-3*np.cos(d)-3.5) - 3*np.cos(b)*np.sin(d)) + np.sin(a)*np.sin(c)*(3*np.cos(d)+3.5)
    z = np.cos(b)*np.cos(c)*(3*np.cos(d)+3.5) - 3*np.sin(b)*np.sin(d) + 2.5
    end_effector = np.array([x,y,z])
    return end_effector

  # Calculate the robot Jacobian
  def calculate_jacobian(self,joints):
    a,b,c,d = joints
    j11 = (-3.5-3*np.cos(d))*np.sin(a)*np.sin(c) + np.cos(a)*(np.cos(c)*(3.5+3*np.cos(d))*np.sin(b) + 3*np.cos(b)*np.sin(d))
    j12 = np.sin(a)*(np.cos(b)*np.cos(c)*(3*np.cos(d)+3.5)-3*np.sin(b)*np.sin(d))
    j13 = -3*np.sin(a)*np.sin(b)*np.cos(d)*np.sin(c) - 3.5*np.sin(a)*np.sin(b)*np.sin(c) + 3*np.cos(a)*np.cos(d)*np.cos(c) + 3.5*np.cos(a)*np.cos(c)
    j14 = np.sin(a)*(3*np.cos(b)*np.cos(d)-3*np.sin(b)*np.cos(c)*np.sin(d)) - 3*np.cos(a)*np.sin(c)*np.sin(d)
    j21 = np.sin(a)*(np.sin(b)*np.cos(c)*(3*np.cos(d)+3.5)+3*np.cos(b)*np.sin(d)) + np.cos(a)*np.sin(c)*(3*np.cos(d)+3.5)
    j22 = np.cos(a)*(np.cos(b)*np.cos(c)*(-3*np.cos(d)-3.5)+3*np.sin(b)*np.sin(d))
    j23 = 3*np.cos(a)*np.sin(b)*np.cos(d)*np.sin(c) + 3.5*np.cos(a)*np.sin(b)*np.sin(c) + 3*np.sin(a)*np.cos(d)*np.cos(c) + 3.5*np.sin(a)*np.cos(c)
    j24 = np.cos(a)*(3*np.sin(b)*np.cos(c)*np.sin(d)-3*np.cos(b)*np.cos(d)) - 3*np.sin(a)*np.sin(c)*np.sin(d)
    j31 = 0
    j32 = np.sin(b)*np.cos(c)*(-3*np.cos(d)-3.5) - 3*np.cos(b)*np.sin(d)
    j33 = np.cos(b)*(-3*np.cos(d)-3.5)*np.sin(c)
    j34 = -3*(np.cos(b)*np.cos(c)*np.sin(d) + np.sin(b)*np.cos(d))
    jacobian = np.array([[j11,j12,j13,j14],[j21,j22,j23,j24],[j31,j32,j33,j34]])
    return jacobian

  # Closed control of the joints
  def control_closed(self):
    # P gain
    K_p = np.array([[10,0,0],[0,10,0],[0,0,10]])
    # D gain
    K_d = np.array([[0.1,0,0],[0,0.1,0],[0,0,0.1]])
    # estimate time step
    cur_time = np.array([rospy.get_time()])
    dt = cur_time - self.time_previous_step
    self.time_previous_step = cur_time
    # robot end-effector position
    pos = self.end_effector_sub.data
    # desired trajectory
    pos_d = self.target_sub.data
    # TO DO Smmoothig

    # estimate derivative of error
    self.error_d = ((pos_d - pos) - self.error)/dt
    # estimate error
    self.error = pos_d-pos
    # estimate initial value of joints
    q = self.joints_sub.data 
    # calculating the psudeo inverse of Jacobian
    J_inv = np.linalg.pinv(self.calculate_jacobian(q)) 
    # control input (angular velocity of joints)
    dq_d =np.dot(J_inv, ( np.dot(K_d,self.error_d.transpose()) + np.dot(K_p,self.error.transpose()) ) )
    # control input (angular position of joints)
    q_d = q + (dt * dq_d)
    return q_d


  def callback(self,data):
    
    # compare the estimated position of robot end-effector calculated by forward kinematics and image
    x_e = self.forward_kinematics(self.joints_sub.data)
    x_e_image = self.end_effector_sub.data
    print(x_e, x_e_image)
    self.end_effector=Float64MultiArray()
    self.end_effector.data= x_e

    # send control commands to joints
    q_d = self.control_closed()
    self.joint1=Float64()
    self.joint1.data= q_d[0]
    self.joint2=Float64()
    self.joint2.data= q_d[1]
    self.joint3=Float64()
    self.joint3.data= q_d[2]
    self.joint4=Float64()
    self.joint4.data= q_d[3]

    # Publish the results
    self.end_effector_pub.publish(self.end_effector)
    self.robot_joint1_pub.publish(self.joint1)
    self.robot_joint2_pub.publish(self.joint2)
    self.robot_joint3_pub.publish(self.joint3)
    self.robot_joint4_pub.publish(self.joint4)
    rate = rospy.Rate(50)
    rate.sleep()

def main(args):
  rc = robot_control()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)


