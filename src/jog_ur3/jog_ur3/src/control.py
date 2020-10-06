#!/usr/bin/env python

import roslib
import rospy
import math
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Float64MultiArray, Float64, Bool, Int32
import numpy as np


class UR3_control(object):
    def __init__(self):
        rospy.init_node('UR3Control', anonymous = True)
        self.image_xy_sub = rospy.Subscriber('cur_xy', Float64MultiArray, self.Image_xy_CB)
        self.ft_sub = rospy.Subscriber('ft_data', Twist, self.ft_CB)
        self.ee_pos_sub = rospy.Subscriber('cur_pos', Point, self.ee_pos_CB)
        self.docked_sub = rospy.Subscriber('docked_flg', Bool, self.docked_CB)
        self.init_sub = rospy.Subscriber('init_flg', Bool, self.init_CB)
        self.track_sub = rospy.Subscriber('track_flag', Int32, self.track_CB)
        self.ee_pos_Pub = rospy.Publisher('goal_pos', Twist, queue_size=1)

        self.docked_flg = 0
        self.track_flg = 0
        self.init_flg = 0
        self.target_vec = np.zeros((2), dtype=float)
        self.pixl2mm = 0.283
        self.ft_vec = np.zeros((3), dtype = float)
        self.force_goal = np.zeros((3), dtype = float)
        self.force_goal[2] = -5.0

        self.cur_pos = np.zeros((3), dtype = float)
        self.Image_ctrl_pixl = np.zeros((2), dtype = float)
        self.Image_ctrl_pixl[0] = 250
        self.Image_ctrl_pixl[1] = 159
        self.dt = 0.1
        self.track_msg_hist = np.ones((5), dtype = int)*2
        self.error_force_acc = np.zeros((3), dtype = float)
        self.last_error_force = np.zeros((3), dtype = float)
        self.last_error_linear = np.zeros((2), dtype = float)
        self.error_linear_acc = np.zeros((2), dtype = float)

    def docked_CB(self, msg):
        self.docked_flg = msg.data

    def track_CB(self, msg):
        self.track_msg_hist[0:4]=self.track_msg_hist[1:5]
        self.track_msg_hist[4] = msg.data

        start_flg = self.check_start(self.track_msg_hist);

        self.track_flg = start_flg if start_flg!=-1 else self.track_flg
        print(self.track_flg)

    def init_CB(self, msg):
        self.init_flg = msg.data

    def Image_xy_CB(self, msg):
        self.target_vec[0] = (msg.data[0] - self.Image_ctrl_pixl[0]) * self.pixl2mm
        self.target_vec[1] = (msg.data[1]- self.Image_ctrl_pixl[1]) * self.pixl2mm

    def ft_CB(self, msg):
        self.ft_vec[0] = np.round(msg.linear.x,1)
        self.ft_vec[1] = np.round(msg.linear.y,1)
        self.ft_vec[2] = np.round(msg.linear.z,1)
        self.controller()


    def ee_pos_CB(self, msg):
        self.cur_pos[0] = np.round(msg.x,4)
        self.cur_pos[1] = np.round(msg.y,4)
        self.cur_pos[2] = np.round(msg.z,4)

    def check_start(self, hist):
        n = len(hist)
        count_start = 0
        count_halt = 0
        for i in range(n):
            if hist[i] == -2:
                count_start +=1
            elif hist[i] == 2:
                count_halt +=1
        if count_halt==n:
            return 0;
        elif count_start ==n:
            return 1;
        else:
            return -1;


    def rotation_matrix_from_vectors(self, vec1, vec2):
        # Find the rotation matrix that aligns vec1 to vec2
        # param vec1: A 3d "source" vector
        # param vec2: A 3d "destination" vector
        # return rot: A transform matrix (3x3) which when applied to vec1, aligns it with vec2.
        if np.array_equal(vec1, np.array([0, 0, 0])) or np.array_equal(vec1, np.arrya([0, 0, -1])):
            vec1=vec2

        a = np.reshape(vec1 / np.linalg.norm(vec1),[1,3])
        b = np.reshape(vec2 / np.linalg.norm(vec2), [1,3])
        v = np.cross(a, b)
        c = np.dot(a, b)
        # s = norm(v);
        kmat = np.arrya([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
        rotation_matrix = np.eye(3, dtype=int) + kmat + kmat*kmat  / (1+c)
        return rotation_matrix


    def cal_RPY(self, rot):
        r21 = rot[1][0]
        r11 = rot[0][0]
        r31 = rot[2][0]
        r32 = rot[2][1]
        r33 = rot[2][2]

        r = np.arctan2(r32, r33);
        p = np.arctan2(-r31, np.sqrt(r32**2+r33**2));
        y = np.arctan2(r21, r11)

        return r,p,y


    def orien_align(self, vec1, vec2):
        rot = self.rotation_matrix_from_vectors(vec1, vec2)
        roll, pitch, yaw = self.cal_RPY(rot)
        return roll, pitch, yaw


    def compare_vel(self, diff, step):
        if np.abs(diff) > step:
            out = step * np.sign(diff)
        else:
            out = 0
        return out


    def vel_smooth(self, vel_last, vel_in ,step):
        diff = vel_in-vel_last
        linear_x = vel_last(1) + compare_vel(diff(1), step)
        linear_y = vel_last(2) + compare_vel(diff(2), step)
        linear_z = vel_last(3) + compare_vel(diff(3), step)
        angular_x = 0
        angular_y = 0
        angular_z = 0
        return linear_x, linear_y, linear_z, angular_x, angular_y, angular_z


    def Linear_PID(self, goal, cur, Kp, Ki, Kd, error_in, last_in, dt):
        # error_* is the integral historical error
        # x_axis = 0 only control y, z axis
        e = goal - cur; # difference between goal and current value
        # e[0] = 0
        error_out = error_in + e * dt; # update integral historical error
        last_out = e;
        output = Kp * e + Ki * error_out - Kd/dt * (e - last_in); #PID controller

        linear_X = 0; # scaler(output[0], 0.1);
        linear_Y = self.scaler(output[0], 0.1);
        linear_Z = self.scaler(output[1], 0.1);

        return linear_X, linear_Y, linear_Z, error_out,last_out

    def Angular_PID(self, dif_RPY, Kp, Ki, Kd, error_in, last_in, dt):
        # error_* is the integral historical error

        error_out = error_in + dif_RPY * dt; #update integral historical error
        last_out = dif_RPY;
        output = Kp * dif_RPY + Ki* error_out - Kd/dt * (dif_RPY - last_in); # PID controller

        angular_X = 0;
        angular_Y = self.scaler(output[2], 0.1);
        angular_Z = 0; #self.scaler(output[1], 0.1);
        return angular_X, angular_Y, angular_Z, error_out,last_out

    def Force_PID(self, goal_force, cur_force, Kp, Ki, Kd, error_in, last_in, dt):
    #   stf_factor = [35e4, 35e4, 30e4]; %N/m
        e = goal_force[2] - cur_force[2];
        error_out = error_in + e;
        last_out = e;
        output = Kp * e  + Ki* error_out - Kd/dt * (e - last_in); # PID controller
        linear_fX = self.scaler(output[0],1);
        return linear_fX, error_out,last_out

    def scaler(self, input, weight):
        # scale the output in range of [-1, 1]
        input = np. round(input,3) * weight;
        if input >= 1:
            output =1
        elif 0< input and input <=0.05:
            output = 0.05
        elif input < 0 and input >= -0.05:
            output = -0.05
        elif input <=-1:
            output = -1
        else:
            output = input
        return output

    def controller(self):
        if self.track_flg:
            [linear_fX, self.error_force_acc ,self.last_error_force] = self.Force_PID(self.force_goal, self.ft_vec, 0, 0, 0, self.error_force_acc, self.last_error_force, self.dt)#0.05, 0.005

            goalMsg = Twist()
            [goalMsg.linear.x, goalMsg.linear.y, goalMsg.linear.z, self.error_linear_acc,self.last_error_linear] = self.Linear_PID(self.target_vec, np.array([0.0,0.0]), 2, 0, 0.01, self.error_linear_acc, self.last_error_linear, self.dt);
            goalMsg.linear.x = linear_fX;
            goalMsg.angular.x=0;
            goalMsg.angular.y=0;
            goalMsg.angular.z=0;

            self.ee_pos_Pub.publish(goalMsg)


Ur3Control = UR3_control()
rospy.spin()
