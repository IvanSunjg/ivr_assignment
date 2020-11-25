#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Int16MultiArray
from cv_bridge import CvBridge, CvBridgeError


class image_converter:

  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('image_processing', anonymous=True)
    # initialize a publisher to send images from camera1 to a topic named image_topic1
    self.image_pub1 = rospy.Publisher("image_topic1",Image, queue_size = 1)
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw",Image,self.callback1)
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()
    # initialize publishers to send the y and z coordinates to the robot
    self.y_z_pub = rospy.Publisher("/robot/y_z", Int16MultiArray, queue_size=10)


  def detect_red(self,image):
    # Isolate the blue colour in the image as a binary image
    mask = cv2.inRange(image, (0, 0, 100), (0, 0, 255))
    # This applies a dilate that makes the binary region larger (the more iterations the larger it becomes)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations=3)
    # Obtain the moments of the binary image
    M = cv2.moments(mask)
    # Calculate pixel coordinates for the centre of the blob
    # M['m00'] ==  0 means that the red is invisible
    if M['m00'] == 0:
      # We can only obtain the y coordiante now
      return np.array([392,0])
    cy = int(M['m10'] / M['m00'])
    cz = int(M['m01'] / M['m00'])
    return np.array([cy, cz])

  def detect_green(self,image):
    # Isolate the blue colour in the image as a binary image
    mask = cv2.inRange(image, (0, 100, 0), (0, 255, 0))
    # This applies a dilate that makes the binary region larger (the more iterations the larger it becomes)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations=3)
    # Obtain the moments of the binary image
    M = cv2.moments(mask)
    # Calculate pixel coordinates for the centre of the blob
    # M['m00'] ==  0 means that the green is invisible now
    if M['m00'] == 0:
      # We can only obtain the y coordiante now
      return np.array([392, 0])
    cy = int(M['m10'] / M['m00'])
    cz = int(M['m01'] / M['m00'])
    return np.array([cy, cz])

  def detect_blue(self,image):
    # Isolate the blue colour in the image as a binary image
    mask = cv2.inRange(image, (100, 0, 0), (255, 0, 0))
    # This applies a dilate that makes the binary region larger (the more iterations the larger it becomes)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations=3)
    # Obtain the moments of the binary image
    M = cv2.moments(mask)
    # M['m00'] ==  0 means that the green is invisible now
    if M['m00'] == 0:
      # We can only obtain the y coordiante now
      return np.array([392, 0])
    # Calculate pixel coordinates for the centre of the blob
    cy = int(M['m10'] / M['m00'])
    cz = int(M['m01'] / M['m00'])
    return np.array([cy, cz])

  def detect_target(self,image):
    mask_orange = cv2.inRange(image, (5, 50, 50), (11, 255, 255))
    ret, thresh = cv2.threshold(mask_orange, 127, 255, cv2.THRESH_BINARY)
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if(len(contours)>=2):
      c = max(contours,key=len)
      (x, y), radius = cv2.minEnclosingCircle(c)
      return np.array([int(x), int(y)])
    else:
      if(len(contours[0])<15):
        return np.array([392, 0])
      else:
        kernel = np.ones((3, 3), np.uint8)
        erosion = cv2.erode(mask_orange, kernel, iterations=1)
        ret, thresh = cv2.threshold(erosion, 127, 255, cv2.THRESH_BINARY)
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        while(len(contours)!=2):
          erosion = cv2.erode(erosion, kernel, iterations=1)
          ret, thresh = cv2.threshold(erosion, 127, 255, cv2.THRESH_BINARY)
          contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        c = max(contours, key=len)
        (x, y), radius = cv2.minEnclosingCircle(c)
        return np.array([int(x), int(y)])

  # Recieve data from camera 1, process it, and publish
  def callback1(self,data):
    # Recieve the image
    try:
      self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    cv2.imshow("", self.cv_image1)
    cv2.waitKey(1)
    
    # Uncomment if you want to save the image
    #cv2.imwrite('image_copy_1.png', self.cv_image1)

    #im1=cv2.imshow('window1', self.cv_image1)

    # Publish the results
    try: 
      self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.cv_image1, "bgr8"))
    except CvBridgeError as e:
      print(e)

    self.red = self.detect_red(self.cv_image1)
    self.green = self.detect_green(self.cv_image1)
    self.blue = self.detect_blue(self.cv_image1)
    self.target = self.detect_target(self.cv_image1)

    self.pub = Int16MultiArray()
    self.pub.data = np.array([self.red[0],self.red[1],self.green[0],
                              self.green[1],self.blue[0],self.blue[1],
                              self.target[0],self.target[1]])

    #Publish the y and z coordinates
    self.y_z_pub.publish(self.pub)


# call the class
def main(args):
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)


