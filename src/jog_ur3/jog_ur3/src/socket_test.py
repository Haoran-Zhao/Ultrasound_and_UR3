#!/usr/bin/env python
import roslib
import rospy
import math
import socket
import geometry_msgs.msg
from geometry_msgs.msg import Point, Twist


if __name__=='__main__':
    rospy.init_node('tf_listener')
    ft_pub = rospy.Publisher('ft_data', Twist, queue_size=1)
    HOST = '192.168.1.4'
    PORT = 63351
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        print("Connecting to " + HOST)
        s.connect((HOST,PORT))
    except:
        print("NO connection")

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            twist = Twist()
            data = s.recv(1024)
            data = data.replace("(", "")
            data = data.replace(")", "\n")
            data = data.split(" , ")
            ft_data = [float(data[0]), float(data[1]), float(data[2])]
            # print(repr(ft_data))
            # rospy.loginfo("ft_data: {}".format(ft_data))
            twist.linear.x = ft_data[0]
            twist.linear.y = ft_data[1]
            twist.linear.z = ft_data[2]
            ft_pub.publish(twist)
        except KeyboardInterrupt:
            s.close

    s.close
    rate.sleep()
