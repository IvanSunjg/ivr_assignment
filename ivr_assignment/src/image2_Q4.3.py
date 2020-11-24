#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Int16MultiArray
from cv_bridge import CvBridge, CvBridgeError


class image_converter:

  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('image_processing', anonymous=True)
    # initialize a publisher to send images from camera2 to a topic named image_topic2
    self.image_pub2 = rospy.Publisher("image_topic2",Image, queue_size = 1)
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw", Image, self.callback2)
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()
    # initialize subscriber to receive the y z coordinates
    self.x_z_pub = rospy.Publisher("/robot/x_z",Int16MultiArray, queue_size = 10)


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


  #This function is used for task 4.3
  def detect_black(self,image):
    # Isolate the blue colour in the image as a binary image
    mask = cv2.inRange(image, (0, 0, 0), (180, 255, 50))
    #To find the "red" and "green" spheres
    circles = cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT, dp=0.9,
                               minDist=0.8, maxRadius=17, param1=100, param2=6)
    circles = np.int16(np.around(circles))

    # helper functions
    def eucDis(u,v):
        return np.sqrt((u-v).dot(u-v))

    def isBlue(u,t=45):
        blue = np.array([396,471])
        return eucDis(blue,u) < t

    def isYellow(u,t=45):
        yellow = np.array([399,529])
        return eucDis(yellow,u) < t

    
    # We can only return the full data if we have detected the two "red" and "green" spheres.
    if circles is not None and len(circles[0]) >= 2:
      print("more than 2 points")
      centers = []
      t = 45
      while len(centers) < 2:
        centers = [i for i in circles[0,:] if (not isBlue(np.array([i[0],i[1]]),t)) and (not isYellow(np.array([i[0],i[1]]),t))]
        t -= 5
      # compare the distances of two tagets and blue jooint
      # calculate the euclidean distance
      distance = np.array([eucDis(np.array([396,471]),np.array([target[0],target[1]]))for target in centers])
      red = np.argmax(distance)
      green = np.argmin(distance)
      return [centers[red][0],centers[red][1], centers[green][0],centers[green][1]]
    
    # This means we only manage to find one single circle
    elif (circles is not None and len(circles[0]) == 1):
      print("single points")
      circles = np.uint16(np.around(circles))
      return [circles[0][0][0],circles[0][0][1],circles[0][0][0],circles[0][0][1]]
    
    # This means we do not manage to find any circle
    # We return the coordinate of the blue sphere
    else:
      print("no points")
      return [399,472,399,472]


  # Recieve data, process it, and publish
  def callback2(self,data):
    # Recieve the image
    try:
      self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    # Publish the results
    try: 
      self.image_pub2.publish(self.bridge.cv2_to_imgmsg(self.cv_image2, "bgr8"))
    except CvBridgeError as e:
      print(e)

    sp = self.detect_black(self.cv_image2)

    # draw detected red circle
    cv2.circle(self.cv_image2, (sp[0], sp[1]), 10, (0, 255, 0), 2)
    # draw the center of the detected red circle
    cv2.circle(self.cv_image2, (sp[0], sp[1]), 2, (0, 0, 255), 2)
    # draw detected green circle
    cv2.circle(self.cv_image2, (sp[2], sp[3]), 10, (0, 255, 0), 2)
    # draw the center of the detected green circle
    cv2.circle(self.cv_image2, (sp[2], sp[3]), 2, (0, 0, 255), 2)

    cv2.imshow("", self.cv_image2)
    cv2.waitKey(1)
    # Uncomment if you want to save the image
    #cv2.imwrite('image_copy_2.png', self.cv_image2)

    self.red = np.array([sp[0],sp[1]])
    self.green = np.array([sp[2],sp[3]])
    self.blue = np.array([396,471])
    self.target = self.detect_target(self.cv_image2)


    self.pub = Int16MultiArray()
    self.pub.data = np.array([self.red[0],self.red[1],self.green[0],
                              self.green[1],self.blue[0],self.blue[1],
                              self.target[0],self.target[1]])

    #Publish the y and z coordinates
    self.x_z_pub.publish(self.pub)

    rate = rospy.Rate(50)
    rate.sleep()



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


