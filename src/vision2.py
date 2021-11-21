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
from collections import deque 
from sklearn.linear_model import LinearRegression
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
    self.predictedZ = rospy.Publisher("predictedZ",Float64,queue_size = 10)
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
    self.dequelength = 18
    self.prevNZ = deque([0]*self.dequelength,maxlen=self.dequelength)
    self.prevNX = deque([0]*self.dequelength,maxlen=self.dequelength)
    self.prevNY = deque([0]*self.dequelength,maxlen=self.dequelength)
    self.maxDiff = 0.4
    self.Zslope = 1
    self.Zpred = 0 
    self.Xslope = 0
    self.sign = 1 
    self.ySign = 1 
    



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
    
    # HSV does a better job than rgb

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    yMask = cv2.inRange(hsv, (15, 0, 0), (36, 255, 255))
    bMask = cv2.inRange(hsv, (110, 50, 50), (130, 255, 255))
    gMask = cv2.inRange(hsv, (36, 0, 0), (70, 255, 255))
    rMask = cv2.inRange(hsv, (0, 70, 50), (10, 255, 255))

    masks = [gMask,yMask,bMask,rMask]
    kernel = np.ones((5, 5), np.uint8)

    centers = []

    # Calculate the center for each joint
    missing= [False,False,False,False]

    for i,mask in enumerate(masks):
      mask = cv2.dilate(mask, kernel, iterations=3)

      # No contours signifies occulusion
      if(np.sum(mask) == 0):
        missing[i] = True
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
    return centers,missing 

  def combine(self,yz,not_here1,xz,not_here2):
    # averages z coord for now
    combined = []
    for i in range(len(yz)):
      if(not_here1[i] == True): # if a center is missing from cam 1  
        combined.append(np.array((xz[i][0],self.prevCenters[i][1],xz[i][1])))
      elif(not_here2[i] == True): # if center is missing from cam 2, 
        combined.append(np.array((self.prevCenters[i][0],yz[i][0],yz[i][1])))
      else:
        combined.append(np.array((xz[i][0],yz[i][0],(xz[i][1]+yz[i][1])/2)))
	
    return combined

  '''
  returns unit vectors with directions:
  joint 1 to joint2/3
  joint 2/3 to joint4
  joint 4 to end effector 
  '''

  def calcNormVecs(self,centers):
    vecs = []
    for i in range(len(centers) - 1):
      vecs.append((centers[i + 1] - centers[i]) / np.linalg.norm(centers[i + 1] - centers[i]))
    return vecs

  '''
  preforms linear regression on the past <self.dequelength> angles 
  '''
  def linregX(self):
    t = np.array(list(range(1,self.dequelength+1))).reshape((-1, 1))
    X = np.array(self.prevNX)
    model = LinearRegression().fit(t, X)
    X_pref = model.predict(np.array([self.dequelength+1]).reshape((-1,1)))[0]
    
    self.Xpred = X_pref
    self.Xslope = model.coef_[0]

  def linregZ(self):
    t = np.array(list(range(1,self.dequelength+1))).reshape((-1, 1))
    z = np.array(self.prevNZ)
    model = LinearRegression().fit(t, z)
#    print(len(x))
    z_pref = model.predict(np.array([self.dequelength+1+0.5]).reshape((-1,1)))[0]
    

    self.Zpred = z_pref
    self.Zslope = model.coef_[0]

  def linregY(self):
    t = np.array(list(range(1,self.dequelength+1))).reshape((-1, 1))
    Y = np.array(self.prevNY)
    model = LinearRegression().fit(t, Y)
    Y_pref = model.predict(np.array([self.dequelength+1]).reshape((-1,1)))[0]
    

    self.Ypred = Y_pref
    self.Yslope = model.coef_[0]
  

  '''
  force -1 <= num <= 1 so arccos and arcsin does not return error
  '''
  def bound(self,num):
    if num > 1: return 1
    elif num <-1: return -1
    else: return num



  def closestRoot(self,A,B,C):
    # predicts approx what value should the next solution have
    self.linregZ()


    # All possible solutions
    roots = []
    r1 = np.arccos(self.bound(-B/np.sin(-1*np.arccos(C))))
    r2 = np.arccos(self.bound(-B/np.sin(np.arccos(C))))
    r3 = np.arccos(self.bound(-B/np.sin(-1*np.arccos(C))))*-1
    r4 = np.arccos(self.bound(-B/np.sin(np.arccos(C))))*-1
    r5 = np.arcsin(self.bound(A/np.sin(-1*np.arccos(C))))
    r6 = np.arcsin(self.bound(A/np.sin(np.arccos(C))))
    roots.append((r1,abs(r1-self.Zpred)))
    roots.append((r2,abs(r2-self.Zpred)))
    roots.append((r3,abs(r3-self.Zpred)))
    roots.append((r4,abs(r4-self.Zpred)))
    roots.append((r5,abs(r5-self.Zpred)))
    roots.append((r6,abs(r6-self.Zpred)))   
    
    # sort solutions such that index[0] is solution closest to predicted Z
    roots.sort(key=lambda x:x[1])

    return roots

  def angle_fromdot(self,vec1,vec2):
    self.linregY()

    y = np.arccos(np.clip(np.dot(vec1, vec2), -1.0, 1.0))*self.ySign

    if np.count_nonzero(self.prevNY)>=self.dequelength/2:
      y = (self.Ypred + y)/2

    if self.ySign > 0 and y < 0.09 and (np.sign(self.Yslope) == -1):   
        self.ySign *= -1
    # If y is negative and gets close enough to 0 with postive slope, flip sign of next x angle
    elif self.ySign < 0 and y > -0.09 and (np.sign(self.Yslope) == 1):
        self.ySign *= -1

    # j4 = np.arccos(np.clip(np.dot(vec1, vec2), -1.0, 1.0))



    self.prevNY.append(y)

    return y

    
  def angles_rotMat(self,prev,cur,hasMissing):

    a,b,c = prev[0],prev[1],prev[2]
    A,B,C = cur[0],cur[1],self.bound(cur[2])
    
    if hasMissing:
      x = self.prevX
#      x = self.prevNX[self.dequelength-1]
    else:
      x = self.sign*np.arccos(C)
    
    if np.count_nonzero(self.prevNX) >=5: 
      self.linregX()


    # If x is positive and gets close enough to 0 with negative slope, flip sign of next x angle
    if self.sign > 0 and x < 0.09 and (np.sign(self.Xslope) == -1):   
        self.sign *= -1

    # If x is negative and gets close enough to 0 with postive slope, flip sign of next x angle
    elif self.sign < 0 and x > -0.09 and (np.sign(self.Xslope) == 1):
        self.sign *= -1

    # make sure we detected some movement before initializing
    if np.count_nonzero(self.prevNZ) >=self.dequelength/2: 
      roots = self.closestRoot(A,B,C)  
      first= roots[0]
      z = first[0]
#      if first[1] > self.maxDiff:
#        z = self.Zpred
      

    # Begin detection by using quadrant 1 solution
    else: 
      z = np.arccos(self.bound(-B/np.sin(np.arccos(C))))
      

    # add z,x to deque of the last N values 
    self.prevNZ.append(z)
    self.prevNX.append(x)
    self.prevX = x
    self.prevZ = z 
    return round(x,5),round(z,5)

    
  def callback2(self,data):
    # Recieve the image
    try:
      self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    centersXZ,not_here_2 = self.detect_centers(self.cv_image2)
    centersYZ,not_here_1 = self.detect_centers(self.cv_image1)

    self.hasMissing = np.any(not_here_1) == True or np.any(not_here_2) ==True
    centers = self.combine(centersYZ,not_here_1,centersXZ,not_here_2)
    self.prevCenters = centers

    normVecs = self.calcNormVecs(centers)

    self.j3,self.j1 = self.angles_rotMat([0.,0.,1.],normVecs[1],self.hasMissing)
    self.j4 = self.angle_fromdot(normVecs[1],normVecs[2])

    # print(normVecs[1],normVecs[2],self.j1,self.j3)
    # cv2.imshow('window', cv_image)
    # cv2.waitKey(3)

    self.joint1,self.joint3,self.joint4 = Float64(),Float64(),Float64()
    self.joint1.data,self.joint3.data,self.joint4.data = self.j1,self.j3,self.j4
        
    self.joint_angle_1.publish(self.joint1)
    self.joint_angle_3.publish(self.joint3)
    self.joint_angle_4.publish(self.joint4)

    self.predZ = Float64()
    self.predZ.data= self.Zpred
    self.predictedZ.publish(self.predZ)

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
