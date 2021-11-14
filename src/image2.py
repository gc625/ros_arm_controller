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


class image_converter:

  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('image_processing', anonymous=True)
    # initialize a publisher to send images from camera2 to a topic named image_topic2
    self.image_pub2 = rospy.Publisher("image_topic2",Image, queue_size = 1)
<<<<<<< HEAD
    
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw",Image,self.callback2)
    # initialize the bridge between openCV and ROS
    self.time = rospy.Subscriber("joint_publisher/time",Float64,self.get_time)
    self.cur_time = 0.0
    self.bridge = CvBridge()

  def get_time(self,data):
  
    self.cur_time = data
  # Recieve data, process it, and publish
  def callback2(self,data):
    r= rospy.Rate(50)
=======
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw",Image,self.callback2)
    # initialize the bridge between openCV and ROS
    self.time = rospy.Subscriber("joint_publisher/time")

    self.bridge = CvBridge()


  # Recieve data, process it, and publish
  def callback2(self,data):
>>>>>>> 373dee8ed664e5f8b2a877970badf86493cc67ad
    # Recieve the image
    try:
      self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    # Uncomment if you want to save the image
<<<<<<< HEAD
    #print(self.cur_time)
    print(rospy.get_time())
    cv2.imwrite('image_copy'+ str(rospy.get_time()) + '.png', self.cv_image2)
    #im2=cv2.imshow('window2', self.cv_image2)
    #cv2.waitKey(1)
    
=======
    cv2.imwrite('image_copy'+ str(self.time) + '.png', self.cv_image2)
    im2=cv2.imshow('window2', self.cv_image2)
    cv2.waitKey(1)

>>>>>>> 373dee8ed664e5f8b2a877970badf86493cc67ad
    # Publish the results
    try: 
      self.image_pub2.publish(self.bridge.cv2_to_imgmsg(self.cv_image2, "bgr8"))
    except CvBridgeError as e:
      print(e)
<<<<<<< HEAD
    r.sleep()
=======
>>>>>>> 373dee8ed664e5f8b2a877970badf86493cc67ad

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


