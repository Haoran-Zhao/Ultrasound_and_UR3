#!/usr/bin/env python
import roslib
import rospy
import math
import socket
import geometry_msgs.msg
from geometry_msgs.msg import Point, Twist
import numpy as np


def lpf(alpha, ft_data, last):
    last = last + alpha * (ft_data - last)
    return last

def ee_pos_CB(msg):
    global cur_pos
    cur_pos[0] = np.round(msg.x,4)
    cur_pos[1] = np.round(msg.y,4)
    cur_pos[2] = np.round(msg.z,4)


if __name__=='__main__':
    rospy.init_node('lv_server')
    cur_pos = np.array([0.0, 0.0, 0.0])
    ee_pos_sub = rospy.Subscriber('cur_pos', Point, ee_pos_CB)

    HOST = '192.168.1.6'
    PORT = 8089
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    try:
        print("Creating LabView server")
        s.bind((HOST,PORT))
        s.listen(1)
    except:
        print("Failed created")

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            conn, addr = s.accept()
            cmnd = conn.recv(4)  # The default size of the command packet is 4 bytes
            # rospy.loginfo(cmnd)
            data = "{},{},{}".format(cur_pos[0],cur_pos[1],cur_pos[2])
            conn.sendall(data.encode())
        except KeyboardInterrupt:
            s.close

    s.close
    rate.sleep()
