#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import imutils
import cv2
from cv_bridge import CvBridge

class video_publisher():
    def __init__(self):
        rospy.init_node('VideoPublisher', anonymous=True)
        self.VideoRaw = rospy.Publisher('VideoRaw', Image, queue_size=1)
        self.bridge = CvBridge()

    def main(self):
        cap = cv2.VideoCapture('/home/haoran/Desktop/video/gel_pad.avi')

        while cap.isOpened():
            ret, current_frame = cap.read()
            current_frame = imutils.resize(current_frame, width=500)

            if not ret:
                break

            cv2.imshow('frames', current_frame)
            # I want to publish the Canny Edge Image and the original Image
            msg_frame = self.bridge.cv2_to_imgmsg(current_frame,"rgb8") # encoding="passthrough"
            # rospy.loginfo_once('publish frame')
            self.VideoRaw.publish(msg_frame)

            key = cv2.waitKey(20)

            if key==ord('q'):
                break

        cv2.destroyAllWindows()
        cap.release()

VideoPublisher = video_publisher()
VideoPublisher.main()
rospy.spin()
