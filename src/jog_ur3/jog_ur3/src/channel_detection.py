#!/usr/bin/env python
import rospy, sys, numpy as np
import cv2, cv_bridge
import numpy as np
import imutils
from imutils.video import VideoStream, FPS
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image


class ultrasound_detection:
    def __init__(self):
        rospy.init_node("ultrasound_detection", anonymous=False)
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('FrameRaw', Image, self.imageCB)
        self.xy_pub = rospy.Publisher('cur_xy', Float64MultiArray, queue_size=1)
        self.mask = cv2.imread('/home/haoran/UR_ws/src/jog_ur3/jog_ur3/mask/doppler_bg_mask2.png')
        self.mask = cv2.cvtColor(self.mask, cv2.COLOR_BGR2GRAY)
        _, self.mask = cv2.threshold(self.mask, 40, 255, cv2.THRESH_BINARY)
        self.cur_pos = Float64MultiArray()

    def gray_contrast_brightness(self, cur_frame_gray, alpha, beta):
        new_image = cur_frame_gray.copy()
        m, n = new_image.shape
        new_image[:, :] = np.clip(np.ones([1, n]) * alpha * cur_frame_gray + np.ones([1, n])*beta, 0, 255)

        return new_image


    def imageCB(self, msg):
        current_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        current_frame = imutils.resize(current_frame, height=500)

        temp = current_frame.copy()
        current_frame_gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)

        _, thre = cv2.threshold(current_frame_gray, 20, 255, cv2.THRESH_BINARY)
        masked = cv2.bitwise_and(thre, self.mask)
        # cv2.imshow('masked', masked)
        contours, hierarchy = cv2.findContours(masked, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours_sizes = [(cv2.contourArea(cnt), cnt) for cnt in contours]
        if contours_sizes:
            primer = max(contours_sizes, key=lambda x: x[0])[1]
            area = cv2.contourArea(primer)
            if area > 35:
                x, y, w, h = cv2.boundingRect(primer)
                cv2.rectangle(temp, (x, y), (x + w, y + h), (0, 255, 0), 2)
                # rect = cv2.minAreaRect(primer)
                # box = cv2.boxPoints(rect)
                # # convert all coordinates floating point values to int
                # box = np.int0(box)
                text = '({}, {})'.format(x + w / 2, y + h / 2)
                cv2.putText(temp, text, (x - 50, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                # cv2.drawContours(temp, [box], 0, (0, 255, 0), 2)
                self.cur_pos.data = [x + w/2, y + h/2]
                self.xy_pub.publish(self.cur_pos)
        cv2.imshow('img', temp)
        key = cv2.waitKey(1)
        if key & 0xFF == ord('q'):
            cv2.destroyAllWindows()


detection = ultrasound_detection()
rospy.spin()
