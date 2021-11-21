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
    rospy.init_node('joint_publisher', anonymous=True)
    robot_joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
    robot_joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
    robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
    robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)
    rate = rospy.Rate(50)  # 50hz
    # initialize a publisher for end effector target positions
    robot_joints_pub = rospy.Publisher("/robot/joints_position_controller/command", Float64MultiArray, queue_size=10)
    joint_time = rospy.Publisher("time", Float64, queue_size=10)

    bag = rosbag.Bag('test.bag', 'w')
    print(rospy.get_time())
    rospy.get_time()
    t0 = rospy.get_time()	
    
    
    while not rospy.is_shutdown():
      for i in ((x/50) for x in range(int(0*50)+1, int(280*50)+1)):
        rospy.get_time()
        cur_time = rospy.get_time() - t0
        print(cur_time,":",i)
        #y_d = float(6 + np.absolute(1.5* np.sin(cur_time * np.pi/100)))
        j1 = np.pi * np.sin(i * np.pi / 28)
        j3 = np.pi * 0.5 * np.sin(i * np.pi / 20) 
        j4 = np.pi * 0.5 * np.sin(i * np.pi / 18) 
        
	
        joint1=Float64()
        joint1.data= j1
        joint3=Float64()
        joint3.data= j3
        joint4=Float64()
        joint4.data= j4
        runtime = Float64()
        runtime.data = cur_time
	
        joint_angles = Float64MultiArray()
        joint_angles.data = np.array([j1, j3, j4])

        
        joint_time.publish(runtime)
        robot_joints_pub.publish(joint_angles.data)
        robot_joint1_pub.publish(joint1)
        robot_joint3_pub.publish(joint3)
        robot_joint4_pub.publish(joint4)

        rate.sleep()
        #if (cur_time >= 10):
         #   rospy.signal_shutdown("time passed")
	
    bag.close()
# run the code if the node is called
if __name__ == '__main__':
    try:
        joint_pub()
    except rospy.ROSInterruptException:
        pass
