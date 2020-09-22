#!/usr/bin/env python
import roslib
import rospy
import math
import socket
from geometry_msgs.msg import Point, Twist

Class ROSLabview():
    def __init__(self):
        rospy.init_node('socket_srever')
        cur_sub = rospy.Subscriber('cur_pos', Point, self.subCB)
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        port = 12345
        self.s.bind(('', port))
        self.s.listen(5)

    def subCB(self, msg):
        c, addr = self.s.accept()
        print('Connection form', addr)
        data = str(msg.x) + ',' + str(msg.y) + ',' + str(msg.z)
        c.send(bytes(data, 'utf-8'))
        c.close()

Roslabview = ROSLabview()
rospy.spin()
