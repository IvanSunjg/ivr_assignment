#!/usr/bin/env python3

import roslib
import sys
import rospy
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray, Float64



class plot_control:

  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('plot_control', anonymous=True)

    # subscriber for target object position
    self.target_sub = rospy.Subscriber("/target_pos",Float64MultiArray,self.get_target)

    # to start callback
    self.end_effector_sub = rospy.Subscriber("/end_pos",Float64MultiArray,self.callback)

    # for ploting 
    self.tgx_pub = rospy.Publisher("/target_x", Float64, queue_size=10)
    self.tgy_pub = rospy.Publisher("/target_y", Float64, queue_size=10)
    self.tgz_pub = rospy.Publisher("/target_z", Float64, queue_size=10)
    self.efx_pub = rospy.Publisher("/end-eff_x", Float64, queue_size=10)
    self.efy_pub = rospy.Publisher("/end-eff_y", Float64, queue_size=10)
    self.efz_pub = rospy.Publisher("/end-eff_z", Float64, queue_size=10)

    print("finished init")

  # Getter for the subscribing data

  def get_target(self,target): 
    #print("get target")
    self.target = np.array(target.data)


  def callback(self,end_pos):

    # get info
    target = self.target
    end_eff = np.array(end_pos.data)

    print("Target: ",target)
    print("End-eff: ",end_eff)


    # Publish the results for ploting.

    self.tgx_pub.publish(target[0])
    self.tgy_pub.publish(target[1])
    self.tgz_pub.publish(target[2])

    self.efx_pub.publish(end_eff[0])
    self.efy_pub.publish(end_eff[1])
    self.efz_pub.publish(end_eff[2])

    rate = rospy.Rate(50)
    rate.sleep()


def main(args):
  pc = plot_control()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)


