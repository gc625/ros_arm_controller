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

  # Defines subscriber
  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('forward_kinematics', anonymous=True)
    self.joint_1 = rospy.Subscriber("joint_angle_1", Float64, self.callback1)
    self.joint_3 = rospy.Subscriber("joint_angle_3", Float64, self.callback2)
    self.joint_4 = rospy.Subscriber("joint_angle_4", Float64, self.callback3)

  def callback1(self, data):
    self.joint1 = float(data.data)

  def callback2(self, data):
    self.joint3 = float(data.data)

  def callback3(self, data):
    self.joint4 = float(data.data)

    self.m1 = self.makeMatrix(4, self.joint1 + (-np.pi / 2), 0, (np.pi / 2))
    self.m2 = self.makeMatrix(0, self.joint3 + (np.pi / 2), 3.2, (np.pi / 2))
    self.m3 = self.makeMatrix(0, self.joint4, 2.8, 0)

    self.finalFk = (self.m1 * self.m2 * self.m3)
    print('ja1: ',self.joint1)
    print('ja3: ',self.joint3)
    print('ja4: ',self.joint4) 
    print(self.finalFk)

  def makeMatrix(self, d, t, r, a):
      M = np.matrix([
       [np.cos(t), -np.sin(t) * np.cos(a), np.sin(t) * np.sin(a), r * np.cos(t)],
       [np.sin(t), np.cos(t) * np.cos(a), -np.cos(t) * np.sin(a), r * np.sin(t)],
       [0, np.sin(a), np.cos(a), d],
       [0, 0, 0, 1]]
      )
      return M


# call the class
def main(args):
  forward_kin()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()


# run the code if the node is called
if __name__ == '__main__':
  main(sys.argv)
