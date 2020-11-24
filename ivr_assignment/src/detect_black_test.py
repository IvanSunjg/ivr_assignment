import numpy as np
import matplotlib.pyplot as plt
import cv2

img1 = cv2.imread("image1_copy.png",1)
img2 = cv2.imread("image2_copy.png",1)

def detect_black(image):
    # Isolate the black colour in the image as a binary image
    mask = cv2.inRange(image, (0, 0, 0), (180, 255, 50))
    return mask

img_black1 = detect_black(img1)
img_black2 = detect_black(img2)

#kernel = np.ones((3,3), np.uint8)
#gradient1 = cv2.morphologyEx(img_black1, cv2.MORPH_GRADIENT,kernel)
#gradient1 = cv2.morphologyEx(img_black2, cv2.MORPH_GRADIENT,kernel)

circles = cv2.HoughCircles(img_black2, cv2.HOUGH_GRADIENT, dp=1.0, minDist=1, maxRadius=10, param1=100, param2=10)

def eucDis(u,v):
    return np.square((u-v).dot(u-v))

def isBlue(u):
    blue = np.array([399,472])
    return eucDis(blue,u) < 20

def isYellow(u):
    yellow = np.array([399,529])
    return eucDis(yellow,u) < 20

if circles is not None and len(circles) > 0:
    print("Circle detected")
    circles = np.uint16(np.around(circles))
    centers = [i for i in circles[0,:] if (not isBlue(np.array([i[0],i[1]]))) and (not isYellow(np.array([i[0],i[1]])))]
    print("Number of circles: ",j)
    cv2.imshow("",img2)
    cv2.waitKey(3)


