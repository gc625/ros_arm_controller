#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
import os,os.path
from datetime import datetime
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

    # Defines publisher and subscriber
    def __init__(self):

        DIR = 'images'
        self.picDIR = 'images/video'+str(len([name for name in os.listdir(DIR) if os.path.isfile(os.path.join(DIR, name))])+1)
#        now = datetime.now().strftime('%d_%m_%Y_%H_%M_%S')
#        self.picDIR = 'images/video' + str(now)
        os.mkdir(self.picDIR) 
        self.c1DIR = os.path.join(self.picDIR,"camera1")	
        self.c2DIR = os.path.join(self.picDIR,"camera2")
        os.mkdir(self.c1DIR)
        os.mkdir(self.c2DIR)

        # initialize the node named image_processing
        rospy.init_node('image_processing', anonymous=True)
        # initialize a publisher to send images from camera1 to a topic named image_topic1
        self.image_pub1 = rospy.Publisher("image_topic1", Image, queue_size=1)
        # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
        self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw", Image, self.callback1)
        # initialize a publisher to send images from camera2 to a topic named image_topic2
        self.image_pub2 = rospy.Publisher("image_topic2", Image, queue_size=1)
        # initialize a subscriber to recieve messages rom a topic named /robot/camera2/image_raw and use callback function to recieve data
        self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw", Image, self.callback2)
        # initialize the bridge between openCV and ROS
        self.bridge = CvBridge()



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

        # Recieve data, process it, and publish

    def callback2(self, data):
        # Recieve the image
        try:
            self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        image = np.concatenate((self.cv_image1, self.cv_image2), axis=1)

#        im = cv2.imshow('camera1 and camera 2', image)
#        cv2.waitKey(1)
        

        cur_time = rospy.get_time()
        cv2.imwrite(self.c1DIR+'/'+str(cur_time)+'.jpg', self.cv_image1)
        cv2.imwrite(self.c2DIR+'/'+str(cur_time)+'.jpg', self.cv_image2)
        #cv2.imwrite('camera1 and camera 2.jpg', self.cv_image2)

        im = cv2.imshow('camera1 and camera 2', image)
        cv2.waitKey(1)
        cv2.imwrite('camera1.jpg', self.cv_image1)
        cv2.imwrite('camera2.jpg', self.cv_image2)
        cv2.imwrite('camera1 and camera 2.jpg', self.cv_image2)

        # Publish the results
        try:
            self.image_pub2.publish(self.bridge.cv2_to_imgmsg(self.cv_image2, "bgr8"))
        except CvBridgeError as e:
            print(e)

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
