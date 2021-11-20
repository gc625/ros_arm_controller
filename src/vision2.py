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
    
    self.joint_angle_1 = rospy.Publisher("joint_angle_1",Float64, queue_size=10)
    self.joint_angle_3 = rospy.Publisher("joint_angle_3",Float64, queue_size=10)
    self.joint_angle_4 = rospy.Publisher("joint_angle_4",Float64, queue_size=10)
    self.quadrant = rospy.Publisher("quadrant",Float64,queue_size = 10)
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
    self.prevX = 0.1
    self.prevZ = 0 
    self.prevCenters = [np.array([0,0,0]),np.array([0,0,0]),np.array([0,0,0])]
    self.sign = 1 
    

  def callback1(self, data):
    
    # Recieve the image
    try:
    
      self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
#      print(type(self.cv_image1))
    except CvBridgeError as e:
      print(e)

    # Publish the results
    try:
      self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.cv_image1, "bgr8"))
    except CvBridgeError as e:
      print(e)

  def detect_centers(self,image):
    # Mask each joint
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    yMask = cv2.inRange(hsv, (15, 0, 0), (36, 255, 255))
    bMask = cv2.inRange(hsv, (110, 50, 50), (130, 255, 255))
    gMask = cv2.inRange(hsv, (36, 0, 0), (70, 255, 255))
    rMask = cv2.inRange(hsv, (0, 70, 50), (10, 255, 255))
    masks = [gMask,yMask,bMask,rMask]
    kernel = np.ones((5, 5), np.uint8)

    centers = []
    # Calculate the center for each joint
    not_here= [False,False,False,False]
    for i,mask in enumerate(masks):
      mask = cv2.dilate(mask, kernel, iterations=3)
      if(np.sum(mask) == 0):
        not_here[i] = True
        centers.append(np.array([0,0]))
      else:
        M = cv2.moments(mask)
        
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        centers.append(np.array([cx,cy]))
    newX = centers[0][0]
    newY = centers[0][1]

    for i in range(len(centers)):
      centers[i] = np.array([centers[i][0] - newX, -1 * (centers[i][1] - newY)])
    return centers,not_here 

  def combine(self,yz,not_here1,xz,not_here2):
    # averages z coord for now
    combined = []
    for i in range(len(yz)):
      if(not_here1[i] == True): # if a center is missing from cam 1  
        combined.append(np.array((xz[i][0],self.prevCenters[i][1],yz[i][1])))
      elif(not_here2[i] == True): # if center is missing from cam 2, 
        combined.append(np.array((self.prevCenters[i][0],yz[i][0],xz[i][1])))
      else:
        combined.append(np.array((xz[i][0],yz[i][0],(xz[i][1]+yz[i][1])/2)))
	
    return combined

  def calcNormVecs(self,centers):
    vecs = []
    for i in range(len(centers) - 1):
  
      vecs.append((centers[i + 1] - centers[i]) / np.linalg.norm(centers[i + 1] - centers[i]))


    return vecs
  def bound(self,num):
    if num > 1: return 1
    elif num <-1: return -1
    else: return num

  def closestRoot(self,A,B,C):
    roots = []
    r1 = np.arccos(self.bound(-B/np.sin(-1*np.arccos(C))))
    r2 = np.arccos(self.bound(-B/np.sin(np.arccos(C))))
    r3 = np.arccos(self.bound(-B/np.sin(-1*np.arccos(C))))*-1
    r4 = np.arccos(self.bound(-B/np.sin(np.arccos(C))))*-1
    r5 = np.arcsin(self.bound(A/np.sin(-1*np.arccos(C))))
    r6 = np.arcsin(self.bound(A/np.sin(np.arccos(C))))
    roots.append((r1,abs(r1-self.prevZ)))
    roots.append((r2,abs(r2-self.prevZ)))
    roots.append((r3,abs(r3-self.prevZ)))
    roots.append((r4,abs(r4-self.prevZ)))
    roots.append((r5,abs(r5-self.prevZ)))
    roots.append((r6,abs(r6-self.prevZ)))   
    roots.sort(key=lambda x:x[1])
    
    return roots[0][0]
  def angles_rotMat(self,prev,cur,q,hasMissing):
#    print("in here, ")

    sol = [0,0]
    a,b,c = prev[0],prev[1],prev[2]
    A,B,C = cur[0],cur[1],self.bound(cur[2])
    
    if hasMissing:
      x = self.prevX
    else:
      x = self.sign*np.arccos(C)
    sol[0] = x 
#    z = np.arcsin(A/np.sin(x))
    
    if self.sign > 0 and x < 0.09 :
      print(self.prevX, x)
      if (np.sign(self.prevX) == np.sign(x)):   
        self.sign *= -1
       
    elif self.sign < 0 and x > -0.09 :
      print(self.prevX, x)
      if (np.sign(self.prevX) == np.sign(x)): 
        self.sign *= -1
#    z = self.closestRoot(A,B,C)      
    if(q == 1):
      if(self.sign == -1):
        z = self.closestRoot(A,B,C)
        
      else:
        z = np.arccos(self.bound(-B/np.sin(np.arccos(C))))
    
    elif(q == 2): 
      if(self.sign == -1):
        z = np.arcsin(self.bound(A/np.sin(-1*np.arccos(C))))
      else:
      	z = self.closestRoot(A,B,C)
      	
    elif(q == 3):
      if(self.sign == -1):
        z = self.closestRoot(A,B,C)
      else:
        z = self.closestRoot(A,B,C)
      
    else:
      if(self.sign == -1):
        z = self.closestRoot(A,B,C)
      else:
        z = self.closestRoot(A,B,C)


    sol[1] = z
    #
#    for i in range(len(sol)):
#      if sol[i]>2:
#        sol[i] = 2.0
#        print("OVERFLOW+")
#      elif sol[i]< -2 :
#        sol[i] = -2.0   
#        print("OVERFLOW-")
    self.prevX = x
    self.prevZ = z 
    return round(sol[0],5),round(sol[1],5)

    
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
#    print("prev:",self.prev)
    
    
    centersXZ,not_here_2 = self.detect_centers(self.cv_image2)
    centersYZ,not_here_1 = self.detect_centers(self.cv_image1)
#    print(not_here_1,not_here_2)
#    print(np.any(not_here_1 == True), np.any(not_here_2 ==True))
    self.hasMissing = np.any(not_here_1) == True or np.any(not_here_2) ==True
#    print(self.hasMissing)
    centers = self.combine(centersYZ,not_here_1,centersXZ,not_here_2)
#    print(centersYZ, centersXZ)
#    print(centers)
    j23 = centers[2]
    
    self.quad = Float64()
    xpos = j23[0]
    ypos = j23[1]
    if xpos >=0 and ypos >=0:
      q = 1
    elif xpos <=0 and ypos >=0:
      q = 2
    elif xpos <=0 and ypos <=0:
      q = 3
    elif xpos >=0 and ypos <=0:
      q = 4
    else:
      q = 1 
    
    
    self.quad.data = q
    self.quadrant.publish(self.quad)
    
    self.prevCenters = centers
    
    normVecs = self.calcNormVecs(centers)
    print(normVecs[1])

    self.j3,self.j1 = self.angles_rotMat([0.,0.,1.],normVecs[1],q,self.hasMissing)
    print(self.j1,self.j3)

    # print("YZ",centersYZ)
    # print("XZ",centersXZ)


    # cv2.imshow('window', cv_image)
    # cv2.waitKey(3)

    self.joint1 = Float64()
    self.joint3 = Float64()
    self.joint4 = Float64()
    self.joint1.data = self.j1
    self.joint3.data = self.j3
#    self.joint4.data = j4
    
    self.joint_angle_1.publish(self.joint1)
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
