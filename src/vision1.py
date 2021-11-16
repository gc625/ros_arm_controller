#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
from sympy import *
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

    # Defines publisher and subscriber



  def __init__(self):
        # initialize the node named image_processing
    rospy.init_node('image_processing', anonymous=True)


    # initialize a publisher to send messages to a topic named image_topic
    self.image_pub = rospy.Publisher("image_topic",Image, queue_size = 1)
    # initialize a publisher to send joints' angular position to a topic called joints_pos
    self.joint_angle_2 = rospy.Publisher("joint_angle_2",Float64, queue_size=10)
    self.joint_angle_3 = rospy.Publisher("joint_angle_3",Float64, queue_size=10)
    self.joint_angle_4 = rospy.Publisher("joint_angle_4",Float64, queue_size=10)
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_pub1 = rospy.Publisher("image_topic1", Image, queue_size=1)
    # initialize a publisher to send images from camera2 to a topic named image_topic2
    self.image_pub2 = rospy.Publisher("image_topic2", Image, queue_size=1)
    # initialize 2 subscribers to get img data
    self.image_sub = rospy.Subscriber("/camera1/robot/image_raw",Image,self.callback1)
    self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw", Image, self.callback2)
    '''
    given a 2D image, return a List[np.array[(x,y)]] 
    representing the center of the green,yellow, blue,red joints
    in that order. 
    '''

  def callback1(self, data):
    # Recieve the image
    try:
      self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    # Publish the results
    try:
      self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.cv_image1, "bgr8"))
    except CvBridgeError as e:
      print(e)

  def detect_centers(self,image):
    # Mask each joint
    rMask = cv2.inRange(image, (0, 0, 100), (0, 0, 255))
    gMask = cv2.inRange(image, (0, 100, 0), (0, 255, 0))
    bMask = cv2.inRange(image, (100, 0, 0), (255, 0, 0))
    yMask = cv2.inRange(image, (0, 100, 100), (0, 255, 255))

    masks = [gMask,yMask,bMask,rMask]
    kernel = np.ones((5, 5), np.uint8)

    centers = []
    # Calculate the center for each joint
    for mask in masks:
      mask = cv2.dilate(mask, kernel, iterations=3)
      M = cv2.moments(mask)
      cx = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
      centers.append(np.array([cx,cy]))
    newX = centers[0][0]
    newY = centers[0][1]

    for i in range(len(centers)):
      centers[i] = np.array([centers[i][0] - newX, -1 * (centers[i][1] - newY)])
    return centers 

  def combine(self,xz,yz):
    # averages z coord for now
    combined = []
    
    for i in range(len(xz)):
      
      combined.append(np.array((xz[i][0],yz[i][0],(xz[i][1]+xz[i][1])/2)))
#      print(xz[i],yz[i])    	
	
    return combined

  def calcNormVecs(self,centers):
    dist = []
    vecs = []
    for i in range(len(centers) - 1):
        vecs.append((centers[i + 1] - centers[i]) / np.linalg.norm(centers[i + 1] - centers[i]))
        # vecs.append((nodeCoords[i + 1] - nodeCoords[i]) / np.linalg.norm(nodeCoords[i + 1] - nodeCoords[i]))
#        dist.append(np.linalg.norm(centers[i+1]-centers[i]))

    return vecs

  def angles_rotMat(self,prev,cur):
    
    
    a,b,c = prev[0],prev[1],prev[2]

    x, y = symbols('x, y')
    eq1 = Eq(cos(y)*a+sin(y)*sin(x)*b+sin(y)*cos(x)*c, cur[0])
    eq2 = Eq(b*cos(x)-c*sin(x), cur[1])


    sol = nsolve([eq1, eq2], [x, y],[1,1])
    
    return sol
  def callback2(self,data):
    # Recieve the image
    try:
      self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    # Perform image processing task (your code goes here)
    # The image is loaded as cv_imag

    # Uncomment if you want to save the image
    #cv2.imwrite('image_copy.png', cv_image)

    centersYZ = self.detect_centers(self.cv_image1)
    centersXZ = self.detect_centers(self.cv_image2)
    centers = self.combine(centersXZ,centersYZ)
#    print(centersYZ, centersXZ)
#    print(centers)
    normVecs = self.calcNormVecs(centers)
    
#    print(self.angles_rotMat([0,0,1],normVecs[1]))
    j2,j3 = self.angles_rotMat([0,0,1],normVecs[1])
#    _,j4 = self.angles_rotMat(normVecs[1],normVecs[2])

#    print(j2,j3)

    # print("YZ",centersYZ)
    # print("XZ",centersXZ)


    # cv2.imshow('window', cv_image)
    # cv2.waitKey(3)

    self.joint2 = Float64()
    self.joint3 = Float64()
    self.joint4 = Float64()
    self.joint2.data = j2 
    self.joint3.data = j3
#    self.joint4.data = j4
    
    self.joint_angle_2.publish(self.joint2)
    self.joint_angle_3.publish(self.joint3)
#    self.joint_angle_4.publish(joint4)
    
    


    # Publish the results
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.cv_image2, "bgr8"))
        # self.joints_pub.publish(self.joints)
    except CvBridgeError as e:
      print(e)



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

