#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import rosbag
import numpy as np
from std_msgs.msg import Float64MultiArray, Float64


class joint_pub:

    # Defines publisher and subscriber
    # initialize the node named
    rospy.init_node('target_publisher', anonymous=True)
    robot_joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
    robot_joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
    robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
    robot_joint3_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)
    rate = rospy.Rate(50)  # 50hz
    # initialize a publisher for end effector target positions
    target_pos_pub = rospy.Publisher("target_pos", Float64MultiArray, queue_size=10)
    rospy.get_time()
    t0 = rospy.get_time()


    bag = rosbag.Bag('test.bag', 'w')

    
    while not rospy.is_shutdown():
        rospy.get_time()
        cur_time = rospy.get_time() - t0
        
        print("in here",cur_time)
        print(t0)
        #y_d = float(6 + np.absolute(1.5* np.sin(cur_time * np.pi/100)))
        j2 = np.pi * 0.5 * np.sin(cur_time * np.pi / 15)
        j3 = np.pi * 0.5 * np.sin(cur_time * np.pi / 20) 
        j4 = np.pi * 0.5 * np.sin(cur_time * np.pi / 18) 
        
	
        joint2=Float64()
        joint2.data= j2
        joint3=Float64()
        joint3.data= j3
        joint4=Float64()
        joint4.data= j4
        runtime = Float64()
        runtime.data = cur_time
	
        joint_angles = Float64MultiArray()
        joint_angles.data = np.array([j2, j3, j4])
        bag.write('joint2',joint2)
        bag.write('joint3',joint3)
        bag.write('joint4',joint4)
        bag.write('time', runtime)
        robot_joint1_pub.publish(joint2)
        robot_joint2_pub.publish(joint3)
        robot_joint3_pub.publish(joint4)
        rate.sleep()
        
        if cur_time == 3:
            rospy.signal_shutdown("3 sec passed")
    bag.close()
# run the code if the node is called
if __name__ == '__main__':
    try:
        joint_pub()
    except rospy.ROSInterruptException:
        pass
