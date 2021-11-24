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
