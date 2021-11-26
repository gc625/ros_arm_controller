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
        self.joint_3 = rospy.Subscriber("joint_angle_3", Float64, self.callback2
                                        )
        self.joint_4 = rospy.Subscriber("joint_angle_4", Float64, self.callback3)
        self.ee_x = rospy.Subscriber("/ee_x", Float64, self.callback4)
        self.ee_y = rospy.Subscriber("/ee_y", Float64, self.callback5)
        self.ee_z = rospy.Subscriber("/ee_z", Float64, self.callback6)
        self.target_pos = rospy.Subscriber("/target_control/target_pos", Float64MultiArray, self.callback7)

        # errors
        self.time_previous_step = np.array([rospy.get_time()], dtype='float64')
        self.time_previous_step2 = np.array([rospy.get_time()], dtype='float64')
        # initialize error and derivative of error for trajectory tracking
        self.error = np.array([0.0, 0.0, 0.0], dtype='float64')
        self.error_d = np.array([0.0, 0.0, 0.0], dtype='float64')
        self.end_effector_pub_x = rospy.Publisher("end_effector_x_prediction", Float64, queue_size=10)
        self.end_effector_pub_y = rospy.Publisher("end_effector_y_prediction", Float64, queue_size=10)
        self.end_effector_pub_z = rospy.Publisher("end_effector_z_prediction", Float64, queue_size=10)
        self.robot_joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
        self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
        self.robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)
        self.rate = rospy.Rate(50)

    def callback1(self, data):
        self.joint1 = float(data.data)

    def callback2(self, data):
        self.joint3 = float(data.data)

    def callback3(self, data):
        self.joint4 = float(data.data)
        
        # getting x,y,z from FK
        coords = self.makeFk(self.joint3, self.joint4, self.joint1)

        print('Coords: ', coords)
        
	# estimated x,y,z vals from FK
        self.ee_pos_fk_x = coords.item(0)
        self.ee_pos_fk_y = coords.item(1)
        self.ee_pos_fk_z = coords.item(2)
        self.end_effector_pub_x.publish(self.ee_pos_fk_x)
        self.end_effector_pub_y.publish(self.ee_pos_fk_y)
        self.end_effector_pub_z.publish(self.ee_pos_fk_z)

        q_d = self.control()
        self.joint1_ik = Float64()
        self.joint1_ik.data = q_d.item(0)
        self.joint3_ik = Float64()
        self.joint3_ik.data = q_d.item(1)
        self.joint4_ik = Float64()
        self.joint4_ik.data = q_d.item(2)

        self.robot_joint1_pub.publish(self.joint1_ik)
        self.robot_joint3_pub.publish(self.joint3_ik)
        self.robot_joint4_pub.publish(self.joint4_ik)



    def callback4(self, data):
        self.ee_x = float(data.data)

    def callback5(self, data):
        self.ee_y = float(data.data)

    def callback6(self, data):
        self.ee_z = float(data.data)

    def callback7(self, data):
        self.target_pos = np.array([float(data.data[0]), float(data.data[1]), float(data.data[2])])

    def makeMatrix(self, d, t, r, a):
        M = np.matrix([
            [np.cos(t), -np.sin(t) * np.cos(a), np.sin(t) * np.sin(a), r * np.cos(t)],
            [np.sin(t), np.cos(t) * np.cos(a), -np.cos(t) * np.sin(a), r * np.sin(t)],
            [0, np.sin(a), np.cos(a), d],
            [0, 0, 0, 1]]
        )
        return M

    def makeFk(self, x, y, z):
        xyz = np.array(
            [[2.8 * np.sin(x) * np.sin(z) * np.cos(y) + 2.8 * np.sin(y) * np.cos(z) + 3.2 * np.sin(z) * np.cos(
                x - np.pi / 2)],
             [-2.8 * np.sin(x) * np.cos(y) * np.cos(z) + 2.8 * np.sin(y) * np.sin(z) - 3.2 * np.cos(
                 z) * np.cos(x - np.pi / 2)],
             [-3.2 * np.sin(x - np.pi / 2) + 2.8 * np.cos(x) * np.cos(y) + 4]])

        return xyz

    def control(self):
        K_p = np.array([[10, 0, 0], [0, 10, 0],[0, 0, 10]])
        K_d = np.array([[3, 0, 0], [0, 3, 0], [0, 0, 3]])
        cur_time = np.array([rospy.get_time()])
        dt = cur_time - self.time_previous_step
        self.time_previous_step = cur_time
        # end effector est position from vision
        pos = np.array([self.ee_x, self.ee_y, self.ee_z])
        pos_d = self.target_pos
        self.error_d = ((pos_d - pos) - self.error) / dt
        self.error = pos_d - pos
        J_inv = np.linalg.pinv(self.calculate_Jacobian(self.joint3, self.joint4, self.joint1))
        q = np.array([self.joint1, self.joint3, self.joint4])
        dq_d = np.dot(J_inv, (np.dot(K_d, self.error_d.transpose()) + np.dot(K_p, self.error.transpose())))
        q_d = q + (dt * dq_d)
        # print('q_d: ', q_d)
        return q_d


    def calculate_Jacobian(self, x, y, z):
        J = np.matrix([
            [0.8 * np.sin(z) * np.cos(x) * np.cos(y) + 3.2 * np.sin(z) * np.cos(x),
             -0.8 * np.sin(x) * np.sin(y) * np.sin(z) + 2.8 * np.cos(y) * np.cos(z),
             0.8 * np.sin(x) * np.cos(y) * np.cos(z) + 3.2 * np.sin(x) * np.cos(z) - 2.8 * np.sin(y) * np.sin(z)],
            [-2.8 * np.cos(x) * np.cos(y) * np.cos(z) - 3.2 * np.cos(x) * np.cos(z),
             2.8 * np.sin(x) * np.sin(y) * np.cos(z) + 2.8 * np.sin(z) * np.cos(y),
             2.8 * np.sin(x) * np.sin(z) * np.cos(y) + 3.2 * np.sin(x) * np.sin(z) + 2.8 * np.sin(y) * np.cos(z)],
            [-2.8 * np.sin(x) * np.cos(y) - 3.2 * np.sin(x), -2.8 * np.sin(y) * np.cos(x), 0],
        ])
        return J


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
