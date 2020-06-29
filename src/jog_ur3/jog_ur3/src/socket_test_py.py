#!/usr/bin/env python
import roslib
import rospy
import math
import socket
import geometry_msgs.msg
from geometry_msgs.msg import Point

HOST = '192.168.1.4'
PORT = 63351
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

try:
    print("Connecting to" + HOST)

    s.connect((HOST,PORT))
    try:
        while 1:
            data = s.recv(1024)
            data = data.replace("(", "")
            data = data.replace(")", "\n")
            print("ft_data: {}".format(data))
    except KeyboardInterrupt:
        s.close
except:
    print("NO connection")
