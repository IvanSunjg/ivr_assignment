import roslib
import sys
import rospy
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray, Float64


class dummy_topics:
    
    def __init__(self):
        rospy.init_node('dummy_topics', anonymous=True)
        # initialize a publisher to send images from camera1 to a topic named image_topic1
        self.joints_pub = rospy.Publisher("joints_ang",Float64MultiArray, queue_size = 1)
        self.target_pub = rospy.Publisher("target_pos",Float64MultiArray, queue_size = 1)
        self.end_eff_pub = rospy.Publisher("end_effector_pos",Float64MultiArray, queue_size = 1)
    
    # Recieve data from camera 1, process it, and publish
    def callback1(self):
        joints = Float64MultiArray()
        joints.data= np.random.rand(4)
        target = Float64MultiArray()
        target.data= np.random.rand(4)
        end_eff = Float64MultiArray()
        end_eff.data= np.random.rand(4)

        self.joints_pub.publish(joints)
        self.target_pub.publish(target)
        self.end_eff_pub.publish(end_eff)

# call the class
def main(args):
  dt = dummy_topics()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)