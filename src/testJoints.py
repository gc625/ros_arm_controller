#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import Float64MultiArray, Float64


def joint_pub():

    # Defines publisher and subscriber
    # initialize the node named
    rospy.init_node('target_publisher', anonymous=True)
    self.robot_joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
    self.robot_joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
    self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
    self.robot_joint3_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)
    rate = rospy.Rate(50)  # 50hz
    # initialize a publisher for end effector target positions
    target_pos_pub = rospy.Publisher("target_pos", Float64MultiArray, queue_size=10)

    t0 = rospy.get_time()
    while not rospy.is_shutdown():
        cur_time = np.array([rospy.get_time()]) - t0
        #y_d = float(6 + np.absolute(1.5* np.sin(cur_time * np.pi/100)))
        j2 = np.pi * 0.5 * np.sin(cur_time * np.pi / 15)
        j3 = np.pi * 0.5 * np.sin(cur_time * np.pi / 20) 
        j4 = np.pi * 0.5 * np.sin(cur_time * np.pi / 18) 
        

        self.joint2=Float64()
        self.joint2.data= j2
        self.joint3=Float64()
        self.joint3.data= j3
        self.joint4=Float64()
        self.joint4.data= j4

        self.robot_joint1_pub.publish(self.joint2)
        self.robot_joint2_pub.publish(self.joint3)
        self.robot_joint3_pub.publish(self.joint4)
        rate.sleep()


# run the code if the node is called
if __name__ == '__main__':
    try:
        target_publisher()
    except rospy.ROSInterruptException:
        pass
