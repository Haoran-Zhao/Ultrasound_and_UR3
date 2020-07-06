#!/usr/bin/env python
import roslib
import rospy
import math
import tf
import geometry_msgs.msg
from geometry_msgs.msg import Point
import PyKDL as kdl

if __name__=='__main__':
    rospy.init_node('tf_listener')
    listener = tf.TransformListener()
    cur_pos_pub = rospy.Publisher('cur_pos', Point, queue_size=1)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
          (trans,rot) = listener.lookupTransform('base_link', 'ee_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        point = Point()
        point.x = trans[0]
        point.y = trans[1]
        point.z = trans[2]
        cur_pos_pub.publish(point)
        roll, pitch, yaw = kdl.Rotation.Quaternion(rot[0],rot[1],rot[2],rot[3]).GetRPY()
        # print(roll, pitch, yaw)
        # rot = PyKDL.Rotation.Quaternion(* [ eval('rot.'+c) for c in 'xyzw'] )
        # print ' '.join( [ str(eval('tor.'+c)) for c in 'xyzw'] )
        # ypr = [ i  / pi * 180 for i in rot.GetEulerZYX() ]
    rate.sleep()
