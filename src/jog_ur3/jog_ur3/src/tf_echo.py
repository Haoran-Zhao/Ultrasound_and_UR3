#!/usr/bin/env python
import roslib
import rospy
import math
import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import Point
from math import pi


if __name__ == '__main__':
    rospy.init_node('tf_echo')

    tfbuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfbuffer)

    cur_pos_pub = rospy.Publisher('cur_pos', Point, queue_size=1)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            trans = tfbuffer.lookup_transform('base_link', 'ee_link', rospy.Time())
            trans = trans.transform
            # print trans
            # rot = PyKDL.Rotation.Quaternion(* [ eval('trans.rotation.'+c) for c in 'xyzw'] )
            # print ' '.join( [ str(eval('trans.rotation.'+c)) for c in 'xyzw'] )
            # ypr = [ i  / pi * 180 for i in rot.GetEulerZYX() ]

            point = Point()
            point.x = trans.translation.x
            point.y = trans.translation.y
            point.z = trans.translation.z
            cur_pos_pub.publish(point)

            # break
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            print "Fail", e



        rate.sleep()
