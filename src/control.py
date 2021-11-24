#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError
import math


class forward_kin:

  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('forward_kinematics', anonymous=True)
    self.joint_1 = rospy.Subscriber("joints_angle_1", Float64MultiArray, self.getActual)
    self.joint_3 = rospy.Subscriber("joints_angle_3", Float64MultiArray, self.getActual)
    self.joint_4 = rospy.Subscriber("joints_angle_4", Float64MultiArray, self.getActual)

  def getActual(self, data):
    self.joint_1 = float(data.data[0])
    self.joint_3 = float(data.data[1])
    self.joint_4 = float(data.data[2])

    self.m1 = self.makeMatrix(4, self.joint_1 + (-np.pi / 2), 0, -np.pi / 2)
    self.m2 = self.makeMatrix(0, self.joint_3 + (-np.pi / 2), 3.2, (np.pi / 2))
    self.m3 = self.makeMatrix(0, self.joint_4, 2.8, 0)
    
    self.finalFk = (self.m1 * self.m2 * self.m3)
    
    print(self.finalFk)
      
  def makeMatrix(d, t, r, a):
      M = np.matrix([
       [np.cos(t), -np.sin(t) * np.cos(a), np.sin(t) * np.sin(a), r * np.cos(t)],
       [np.sin(t), np.cos(t) * np.cos(a), -np.cos(t) * np.sin(a), r * np.sin(t)],
       [0, np.sin(a), np.cos(a), d],
       [0, 0, 0, 1]]
      )
      return M

  # M1 = makeMatrix(4,1.3-np.pi/2,0,-np.pi/2)
  # M2 = makeMatrix(0,1.5+(-np.pi/2),3.2,(np.pi/2))
  # M3 = makeMatrix(0,-1,2.8,0)

  j1, j3, j4 = 1.5, 1, -1.3

  # d t r a
  M1 = makeMatrix(4, j1 + (-np.pi / 2), 0, -np.pi / 2)
  M2 = makeMatrix(0, j3 + (-np.pi / 2), 3.2, (np.pi / 2))
  M3 = makeMatrix(0, j4, 2.8, 0)

  #print(M1)
  #print(M2)
  #print(M3)

  print(M1 * M2 * M3)


# call the class
def main(args):
  fk = forward_kin()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()


# run the code if the node is called
if __name__ == '__main__':
  main(sys.argv)
