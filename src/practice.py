#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
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
        self.image_pub = rospy.Publisher("image_topic", Image, queue_size=1)
        # initialize a publisher to send joints' angular position to a topic called joints_pos
        self.joint_angle_2 = rospy.Publisher("joint_angle_2", Float64, queue_size=10)
        self.joint_angle_3 = rospy.Publisher("joint_angle_3", Float64, queue_size=10)
        self.joint_angle_4 = rospy.Publisher("joint_angle_4", Float64, queue_size=10)
        # initialize the bridge between openCV and ROS
        self.bridge = CvBridge()
        # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
        self.image_pub1 = rospy.Publisher("image_topic1", Image, queue_size=1)
        # initialize a publisher to send images from camera2 to a topic named image_topic2
        self.image_pub2 = rospy.Publisher("image_topic2", Image, queue_size=1)
        # initialize 2 subscribers to get img data
        self.image_sub = rospy.Subscriber("/camera1/robot/image_raw", Image, self.callback1)
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

    def detect_centers(self, image):
        # Mask each joint
        rMask = cv2.inRange(image, (0, 0, 100), (0, 0, 255))
        gMask = cv2.inRange(image, (0, 100, 0), (0, 255, 0))
        bMask = cv2.inRange(image, (100, 0, 0), (255, 0, 0))
        yMask = cv2.inRange(image, (0, 100, 100), (0, 255, 255))

        masks = [gMask, yMask, bMask, rMask]
        kernel = np.ones((5, 5), np.uint8)

        centers = []
        # Calculate the center for each joint
        for mask in masks:
            mask = cv2.dilate(mask, kernel, iterations=3)
            M = cv2.moments(mask)
            if M['m00'] == 0:
                return (np.array([555, 666])) # dummy for when circle is not visible
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            centers.append(np.array([cx, cy]))
        return centers

    def callback2(self, data):
        # Recieve the image
        try:
            self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        centersYZ = self.detect_centers(self.cv_image1)
        centersXZ = self.detect_centers(self.cv_image2)

        dists = []
        for i in range(3):
            dists.append(np.sum((centersYZ[i] - centersYZ[i + 1]) ** 2))

        lengthGY = 4 / np.sqrt(dists[0])
        lengthYB = 3.2 / np.sqrt(dists[1])
        lengthBR = 2.8 / np.sqrt(dists[2])


        center = lengthGY * centersYZ[0]
        circle1Pos = 0 * centersYZ[1]
        circle2Pos = lengthYB * centersYZ[2]
        circle3Pos = lengthBR * centersYZ[3]

        ja1 = np.arctan2(center - circle1Pos[0], center - circle1Pos[1])
        ja2 = np.arctan2(circle1Pos[0] - circle2Pos[0], circle1Pos[1] - circle2Pos[1]) - ja1
        ja3 = np.arctan2(circle2Pos[0] - circle3Pos[0], circle2Pos[1] - circle3Pos[1]) - ja2 - ja1
        print("angles:")
        print(np.array([ja1, ja2, ja3]))

        # print("YZ",centersYZ)
        # print("XZ",centersXZ)
        print('link 1:')
        print(lengthGY)
        print('link 2:')
        print(lengthYB)
        print('link 3:')
        print(lengthBR)

        # cv2.imshow('window', cv_image)
        # cv2.waitKey(3)

        self.joint_angle_2 = Float64()
        self.joint_angle_3 = Float64()
        self.joint_angle_4 = Float64()
        # self.joint_angle_2 = # TODO
        # self.joint_angle_3 = # TODO
        # self.joint_angle_4 = # TODO

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
