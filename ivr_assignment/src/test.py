import cv2
import numpy as np

# In this method you can focus on detecting the centre of the red circle
def detect_red(image):
    # Isolate the blue colour in the image as a binary image
    mask = cv2.inRange(image, (0, 0, 100), (0, 0, 255))
    # This applies a dilate that makes the binary region larger (the more iterations the larger it becomes)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations=5)
    # Obtain the moments of the binary image
    M = cv2.moments(mask)
    # Calculate pixel coordinates for the centre of the blob
    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])
    return np.array([cx, cy])


# Detecting the centre of the green circle
def detect_green(image):
    mask = cv2.inRange(image, (0, 100, 0), (0, 255, 0))
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations=5)
    M = cv2.moments(mask)
    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])
    return np.array([cx, cy])


# Detecting the centre of the blue circle
def detect_blue(image):
    mask = cv2.inRange(image,(100,0,0),(255,0,0))
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations=5)
    M = cv2.moments(mask)
    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])
    return np.array([cx, cy])


# Detecting the centre of the yellow circle
def detect_yellow(image):
    mask = cv2.inRange(image, (0, 100, 100), (0, 255, 255))
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations=5)
    M = cv2.moments(mask)
    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])
    return np.array([cx, cy])

def pixel2meter(image):
    blue = detect_blue(image)
    green = detect_green(image)
    dist=np.sum((blue-green)**2)
    return 3.5 / np.sqrt(dist)

img_1 = cv2.imread('image_copy_2.png', 1)
#cv2.imshow('window1',img)
yellow = detect_yellow(img_1)
blue = detect_blue(img_1)
green = detect_green(img_1)
red = detect_red(img_1)

#print(yellow)
#print(green)
#print(blue)
#print(red)
yellow_to_blue = blue-yellow
blue_to_green = green-blue
z = np.sqrt((353-396)**2+(457-478)**2)
print(np.arctan2((477-408),z))
z = np.sqrt((477-408)**2+(457-478)**2)
print(np.arctan2(353-396,z))
print(green)
print(blue)
green_to_blue = [396-353,408-477,478-457]
z = [396-353,0,np.sqrt((353-396)**2+(457-478)**2)]
print(np.arctan2(408-477,475-452))
print( np.arccos(np.dot(green_to_blue, z) /
                                    (np.linalg.norm(green_to_blue) * np.linalg.norm(z))))


#(100, 0, 0), (255, 0, 0)
mask_orange = cv2.inRange(img_1,(5,50,50),  (11,255,255))
mask_green = cv2.inRange(img_1,(0, 100, 0), (0, 255, 0))
kernel = np.ones((3, 3), np.uint8)
mask_blue = cv2.dilate(mask_orange, kernel, iterations=5)
#mask_green = cv2.dilate(mask_green, kernel, iterations=5)
#cv2.imshow('window1',mask_blue)
#cv2.imshow('window2',mask_green)

#M = cv2.moments(mask)
#cx = int(M['m10'] / M['m00'])
#cy = int(M['m01'] / M['m00'])

#cv2.waitKey()
#cv2.destroyAllWindows()
#imgray = cv2.cvtColor(mask_orange,cv2.COLOR_BGR2GRAY)

ret, thresh = cv2.threshold(mask_orange,127,255,cv2.THRESH_BINARY)
contours, hierarchy =  cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
print(len(contours))
print(len(contours[0]))
kernel = np.ones((3,3),np.uint8)
erosion = cv2.erode(mask_orange,kernel,iterations=1)
erosion = cv2.erode(erosion,kernel,iterations=1)
#print(len(contours[1]))
#c = max(contours,key=len)
#print(len(c))
#(x,y), radius = cv2.minEnclosingCircle(c)
#center = (int(x),int(y))
#radius = int(radius)
#print(center,radius)
#img = cv2.circle(mask_orange,center,radius,(0,255,0),2)
cv2.imshow('window1',erosion)
cv2.waitKey()
cv2.destroyAllWindows()