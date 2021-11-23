#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
from sympy import *
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64,Float64MultiArray
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
    
    self.joint_angle_2 = rospy.Publisher("joint_angle_2",Float64, queue_size=10)
    self.joint_angle_3 = rospy.Publisher("joint_angle_3",Float64, queue_size=10)
    self.joint_angle_4 = rospy.Publisher("joint_angle_4",Float64, queue_size=10)

    self.joint_2_error = rospy.Publisher("joint_2_error",Float64, queue_size=10)
    self.joint_3_error = rospy.Publisher("joint_3_error",Float64, queue_size=10)
    self.joint_4_error = rospy.Publisher("joint_4_error",Float64, queue_size=10)
    
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_pub1 = rospy.Publisher("image_topic1", Image, queue_size=1)
    # initialize a publisher to send images from camera2 to a topic named image_topic2
    self.image_pub2 = rospy.Publisher("image_topic2", Image, queue_size=1)
    # initialize 2 subscribers to get img data
    self.image_sub = rospy.Subscriber("/camera1/robot/image_raw",Image,self.callback1)
    self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw", Image, self.callback2)
    # self.actual_joints = rospy.Subscriber("joints_actual",Float64MultiArray,self.getActual)
    
    # maybe no need

    self.j2 = 0
    self.j3 = 0
    self.prevCenters = [np.array([0,0,0]),np.array([0,0,0]),np.array([0,0,0])]
    


    self.dequelength = 18
    self.prevNY2 = deque([0]*self.dequelength,maxlen=self.dequelength)
    self.prevNX = deque([0]*self.dequelength,maxlen=self.dequelength)
    self.prevNY = deque([0]*self.dequelength,maxlen=self.dequelength)

    self.maxDiff = 0.4

    self.Zslope = 1
    self.Zpred = 0 
    self.Yslope = 0
    self.Ypred = 0
    self.Y2slope = 0
    self.Y2pred = 0


    self.joint_2_actual = 0.
    self.joint_3_actual = 0.
    self.joint_4_actual = 0.


    
  def getActual(self,data):
    self.joint_1_actual = float(data.data[0])
    self.joint_3_actual = float(data.data[1])
    self.joint_4_actual = float(data.data[2])


  def callback1(self, data):
    
    # Recieve the image
    try:
    
      self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
      print(type(self.cv_image1))
    except CvBridgeError as e:
      print(e)

    # Publish the results
    try:
      self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.cv_image1, "bgr8"))
    except CvBridgeError as e:
      print(e)

  def detect_centers(self,image):
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

  def calcNormVecs(self,centers):
    vecs = []
    for i in range(len(centers) - 1):
      vecs.append((centers[i + 1] - centers[i]) / np.linalg.norm(centers[i + 1] - centers[i]))
    return vecs

  def linregX(self):
    t = np.array(list(range(1,self.dequelength+1))).reshape((-1, 1))
    X = np.array(self.prevNX)
    model = LinearRegression().fit(t, X)
    X_pref = model.predict(np.array([self.dequelength+1]).reshape((-1,1)))[0]
    
    self.Xpred = X_pref
    self.Xslope = model.coef_[0]
  
  def linregY(self):
    t = np.array(list(range(1,self.dequelength+1))).reshape((-1, 1))
    Y = np.array(self.prevNY)
    model = LinearRegression().fit(t, Y)
    Y_pref = model.predict(np.array([self.dequelength+1]).reshape((-1,1)))[0]
    self.Ypred = Y_pref
    self.Yslope = model.coef_[0]

  def linregY2(self):
    t = np.array(list(range(1,self.dequelength+1))).reshape((-1, 1))
    Y2 = np.array(self.prevNY2)
    model = LinearRegression().fit(t, Y2)
    y2_pref = model.predict(np.array([self.dequelength+1+0.5]).reshape((-1,1)))[0]
    self.Y2pred = y2_pref
    self.Y2slope = model.coef_[0]

  def bound(self,num):
    if num > 1: return 1
    elif num <-1: return -1
    else: return num


  def closestXRoot(self,B):
    roots = []
    r1 = np.arcsin(self.bound(-B))
    d1 = abs(r1-self.Xpred)
    r2 = -1*np.arcsin(self.bound(-B))
    d2 = abs(r2-self.Xpred)
    roots.append((r1,d1))
    roots.append((r2,d2))
    roots.sort(key=lambda x:x[1])
    CalculatedX,firstDiff = roots[0][0], roots[0][1] 
       
    if firstDiff > self.maxDiff:
      x = (3/5)*self.Xpred + (2/5)*CalculatedX
    else:
      x = (1/3)*self.Xpred + (2/3)*CalculatedX

    return x 


  def closestYRoot(self,A,C,x):
    roots = []
    r1 = np.arcsin(self.bound(A/np.cos(x)))
    d1 = abs(r1-self.Ypred)
    r2 = -1*np.arcsin(self.bound(A/np.cos(x)))
    d2 = abs(r2-self.Ypred)
    r3 = np.arccos(self.bound(C/np.cos(x)))
    d3 = abs(r3-self.Ypred)
    r4 = np.arccos(self.bound(-1*C/np.cos(x)))
    d4 = abs(r4-self.Ypred)
    roots.append((r1,d1))
    roots.append((r2,d2))
    roots.append((r3,d3))
    roots.append((r4,d4))
    
    roots.sort(key=lambda x:x[1])
    CalculatedY,firstDiff = roots[0][0], roots[0][1] 

    if firstDiff > self.maxDiff:
      y = (4/5)*self.Ypred + (1/5)*CalculatedY
    else:
      y = (2/5)*self.Ypred + (3/5)*CalculatedY

    return y


  def closestY2Root(self):
    roots = []
    r1 = np.arccos(np.clip(np.dot(vec1, vec2), -1.0, 1.0))
    d1 = abs(r1-self.Y2pred)
    r2 = -1*np.arccos(np.clip(np.dot(vec1, vec2), -1.0, 1.0))
    d2 = abs(r2-self.Y2pred)
    roots.append((r1,d1))
    roots.append((r2,d2))
    roots.sort(key=lambda x:x[1])
    CalculatedY2,firstDiff = roots[0][0], roots[0][1] 
    if firstDiff > self.maxDiff:
      y2 = (4/5)*self.Y2pred + (1/5)*CalculatedY2
    else:
      y2 = (2/5)*self.Y2pred + (3/5)*CalculatedY2

    return y2


  def angle_fromdot(self,vec1,vec2):
    self.linregY2()

    if np.count_nonzero(self.prevNY2)>=self.dequelength/2:
      y2 = self.closestY2Root(vec1,vec2)
    else:
      y2 = np.arccos(np.clip(np.dot(vec1, vec2), -1.0, 1.0))

    self.prevNY2.append(y2)

    return y2

  def angles_rotMat(self,prev,cur,hasMissing):
    
    a,b,c = prev[0],prev[1],prev[2]
    A,B,C = cur[0],cur[1],self.bound(cur[2])


    if np.count_nonzero(self.prevNX) >=5: 
      self.linregX()


    if hasMissing:
      # x = self.prevX
      x = self.Xpred
#      x = self.prevNX[self.dequelength-1]
    else:
      if np.count_nonzero(self.prevNX) >=self.dequelength/2: 
        x = self.closestXRoot(B)
      else:
        x = np.arcsin(self.bound(-B))


    if np.count_nonzero(self.prevNY) >=self.dequelength/2: 
      y = np.arcsin(self.bound(A/np.cos(x)))
      # y = self.closestYRoot(A,C,x)  
    # Begin detection by using quadrant 1 solution
    else: 
      y = np.arcsin(self.bound(A/np.cos(x)))


    self.prevNY.append(y)
    self.prevNX.append(x)
    self.prevX = x
    self.prevY = y
    return round(y,5),round(x,5)



    
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

    self.j2,self.j3 = self.angles_rotMat([0.,0.,1.],normVecs[1],self.hasMissing)
    self.j4 = self.angle_fromdot(normVecs[1],normVecs[2])


    # print("YZ",centersYZ)
    # print("XZ",centersXZ)


    # cv2.imshow('window', cv_image)
    # cv2.waitKey(3)

    self.joint2 = Float64()
    self.joint3 = Float64()
    self.joint4 = Float64()
    self.joint2.data = self.j2 
    self.joint3.data = self.j3
    self.joint4.data = self.j4
    
    self.joint_angle_2.publish(self.joint2)
    self.joint_angle_3.publish(self.joint3)
    self.joint_angle_4.publish(self.joint4)
    
    
    


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

