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

    # subscriber for target object position
    self.target_sub = rospy.Subscriber("/target_pos",Float64MultiArray,self.get_target)
    
    # subscriber for end_effector position
    self.end_effector_sub = rospy.Subscriber("/end_pos",Float64MultiArray,self.get_end_effector)

    # to start callback
    self.end_effector_sub2 = rospy.Subscriber("/end_pos",Float64MultiArray,self.callback)
    
    # initialize a publisher to send joints' angular position to the robot
    self.robot_joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
    self.robot_joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
    self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
    self.robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)

    # initialize for closed control
    self.targets = []
    self.end_poss= []
    self.pre_time = np.array([rospy.get_time()],dtype='float64')
    self.error = np.array([0.0,0.0,0.0],dtype='float64')
    self.d_error = np.array([0.0,0.0,0.0],dtype='float64')
    self.errors = [np.array([0.0,0.0,0.0],dtype="float64") for i in range(10)]
    self.pre_joints = np.array([0.0,0.0,0.0,0.0],dtype='float64')
    print("finished init")

  # Getter for the subscribing data

  def get_target(self,target): 
    #print("get target")
    self.target = np.array(target.data)

  def get_end_effector(self,end_effector): 
    #print("get end_pos")
    self.end_pos = np.array(end_effector.data)

  # Calculate the forward kinematics
  def forward_kinematics(self,joints):
    a,b,c,d = joints
    x = np.sin(a)*(np.sin(b)*np.cos(c)*(3*np.cos(d)+3.5)+3*np.cos(b)*np.sin(d)) + np.cos(a)*np.sin(c)*(3*np.cos(d)+3.5)
    y = np.cos(a)*(np.sin(b)*np.cos(c)*(-3*np.cos(d)-3.5) - 3*np.cos(b)*np.sin(d)) + np.sin(a)*np.sin(c)*(3*np.cos(d)+3.5)
    z = np.cos(b)*np.cos(c)*(3*np.cos(d)+3.5) - 3*np.sin(b)*np.sin(d) + 2.5
    end_effector = np.array([x,y,z])
    return end_effector

  # Calculate the robot Jacobian
  def jacobian(self,joints):
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
  def control_closed(self,end_effector,target):
    # PD parameter
    K_p = np.array([[7,0,0],[0,7,0],[0,0,7]]) # 1
    K_d = np.array([[0.7,0,0],[0,0.7,0],[0,0,0.7]]) # 0.1
    
    # calculate dt
    cur_time = np.array([rospy.get_time()])
    dt = cur_time - self.pre_time
    self.pre_time = cur_time
    
    # get end-effector position and target position
    # smoothing the end_effector position obtained from computer vision is effective
    if len(self.end_poss) > 10:
      self.end_poss.pop(0)
    self.end_poss.append(end_effector)
    pos = np.array(self.end_poss).mean(axis=0)
    # smoothing of the target position is not effective.
    pos_d = target
    
    # calculate derivative of error and error
    self.d_error = ((pos_d - pos) - self.error)/dt
    print("d_error: ",self.d_error)
    self.error = (pos_d - pos)
    print("error: ",self.error)
    
    # get initial value of joints
    q = self.pre_joints
    
    # calculating the psudeo inverse of Jacobian
    J_inv = np.linalg.pinv(self.jacobian(q)) 
    
    # PD-control (angular velocity of joints)
    print("eff_velo: ",K_d.dot(self.d_error.T) + K_p.dot(self.errors[-1].T))
    dq_d = J_inv.dot(K_d.dot(self.d_error.T) + K_p.dot(self.errors[-1].T))
    
    # control input (angular position of joints)
    q_d = q + (dt * dq_d)
    print("Joint move: ",(dt * dq_d))
    self.pre_joints = q_d
    return q_d


  def callback(self,data):

    # Task 3.2: Closed Control

    # send control commands to joints
    target = self.target
    end_eff = self.end_pos

    q_d = self.control_closed(end_eff,target)

    self.joint1=Float64()
    self.joint1.data= q_d[0]
    self.joint2=Float64()
    self.joint2.data= q_d[1]
    self.joint3=Float64()
    self.joint3.data= q_d[2]
    self.joint4=Float64()
    self.joint4.data= q_d[3]

    # Publish the results
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


