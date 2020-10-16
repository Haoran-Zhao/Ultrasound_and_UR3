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
        self.mask = cv2.imread('/home/haoran/UR_ws/src/jog_ur3/jog_ur3/mask/flash_bg_mask.png')
        self.mask = cv2.cvtColor(self.mask, cv2.COLOR_BGR2GRAY)
        self.cur_pos = Float64MultiArray()


    def contrast_brightness(self, cur_frame_gray, alpha, beta):
        new_image = cur_frame_gray.copy()
        m, n = new_image.shape
        new_image[:, :] = np.clip(np.ones([1, n]) * alpha * cur_frame_gray + np.ones([1, n])*beta, 0, 255)

        return new_image

    def find_contour(self, img):
        contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # merge nearby contour -->k nearest neighbour
        contoursRect = []
        for c in contours:
            [x, y, w, h] = cv2.boundingRect(c)
            if not contoursRect:
                contoursRect.append((x, y, w, h))
            else:
                index = -1
                mind = float('inf')
                # find the closest contour landmark in contoursRect
                for i, dims in enumerate(contoursRect):
                    [xr, yr, wr, hr] = dims
                    dist = np.sqrt((x + w / 2 - xr - wr / 2) ** 2 + (y + h / 2 - yr - hr / 2) ** 2)
                    if dist < mind and dist < 50:
                        mind = dist
                        index = i

                if index == -1:
                    contoursRect.append((x, y, w, h))
                else:
                    # merge the closest contour
                    arr = []
                    [xr, yr, wr, hr] = contoursRect[index]
                    arr.append((xr, yr))
                    arr.append((xr + wr, yr + hr))
                    arr.append((x, y))
                    arr.append((x + w, y + h))
                    x, y, w, h = cv2.boundingRect(np.asarray(arr))
                    contoursRect[index] = (x, y, w, h)

        n = len(contoursRect)
        i = 0
        while i < n:
            # refine contour landmarks and merge close contour (selection sort)
            [x, y, w, h] = contoursRect[i]
            index = -1
            mind = float('inf')
            if i + 1 < n:
                for j in range(i + 1, n):
                    [xr, yr, wr, hr] = contoursRect[j]
                    dist = np.sqrt((x + w / 2 - xr - wr / 2) ** 2 + (y + h / 2 - yr - hr / 2) ** 2)
                    if dist < mind and dist < 50:
                        mind = dist
                        index = j
                if index == -1:
                    i += 1
                    continue
                else:
                    arr = []
                    [xr, yr, wr, hr] = contoursRect[index]
                    arr.append((xr, yr))
                    arr.append((xr + wr, yr + hr))
                    arr.append((x, y))
                    arr.append((x + w, y + h))
                    x, y, w, h = cv2.boundingRect(np.asarray(arr))
                    contoursRect[i] = (x, y, w, h)
                    del contoursRect[index]
                    n -= 1
            i += 1

        return contoursRect


    def preprocess(self, img, alpha=2, beta=-50):
        img = self.contrast_brightness(img, alpha, beta)
        _, img = cv2.threshold(img, 30, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for c in contours:
            area = cv2.contourArea(c)

            # Fill very small contours with zero (erase small contours).
            if area < 30 or area > 2500:
                cv2.fillPoly(img, pts=[c], color=0)
                continue

        return img


    def imageCB(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        img = imutils.resize(img, height=500)
        temp = img.copy()
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img = cv2.bitwise_and(img, self.mask)
        img = self.preprocess(img)
        contoursRect = self.find_contour(img)

        contours_sizes = [(cnt[2]*cnt[3], cnt) for cnt in contoursRect]
        if contours_sizes:
            primer = max(contours_sizes, key=lambda x: x[0])[1]
            area = primer[2]*primer[3]
            if area > 35:
                arr = []
                x, y, w, h = primer
                arr.append((x, y))
                arr.append((x + w, y + h))
                x, y, w, h = cv2.boundingRect(np.asarray(arr))
                cv2.rectangle(temp, (x, y), (x + w, y + h), (0, 255, 0), 2)
                # rect = cv2.minAreaRect(primer)
                # box = cv2.boxPoints(rect)
                # # convert all coordinates floating point values to int
                # box = np.int0(box)
                text = '({}, {})'.format(x + w / 2, y + h / 2)
                cv2.putText(temp, text, (x - 50, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                # cv2.drawContours(temp, [box], 0, (0, 255, 0), 2)
                self.cur_pos.data = [x + w/2, y + h/2]
            else:
                self.cur_pos.data = [0, 0]
        else:
            self.cur_pos.data = [0, 0]
        self.xy_pub.publish(self.cur_pos)
        cv2.imshow('img', temp)
        key = cv2.waitKey(1)
        if key & 0xFF == ord('q'):
            cv2.destroyAllWindows()


detection = ultrasound_detection()
rospy.spin()
