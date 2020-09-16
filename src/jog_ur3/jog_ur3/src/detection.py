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

        # initialize the bounding box coordinate of the object we are going to track
        self.initBB = None
        self.lastBB = None
        self.fps = None
        self.target = None
        self.alpha = 1.0  # Simple contrast control
        self.beta = 0    # Simple brightness control
        self.count = 0
        self.fgbg = cv2.bgsegm.createBackgroundSubtractorMOG()
        self.tracker = cv2.TrackerCSRT_create()
        self.cur_pos = Float64MultiArray()

    def csrt_tracker(self, cur_frame):
        (H, W) = cur_frame.shape[:2]

        # check if we are currently tacking an object
        if self.initBB is not None:
            # grab new bounding box coordinates of the object
            (success, box) = self.tracker.update(cur_frame)

            # check to see if the tracking was success
            if success:
                #compare bounding box changing step size, if too big reset tracker
                if self.lastBB:
                    (xl, yl, wl, hl) = [int(v) for v in self.lastBB]
                    (x, y, w, h) = [int(v) for v in box]

                    dist = np.sqrt((x+w/2 - xl-wl/2) ** 2 + (y+h/2 - yl-hl/2) ** 2)
                    if dist > 40:
                        print(dist)
                        print("clear initBB")
                        self.tracker = cv2.TrackerCSRT_create()
                        self.initBB = None
                        self.lastBB = None
                        self.fps = None
                        self.fps = FPS().start()
                        return cur_frame

                    else:
                        self.target = (x, y, w, h)
                        cv2.rectangle(cur_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                else:
                    (x, y, w, h) = [int(v) for v in box]
                    cv2.rectangle(cur_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

                self.lastBB = box
                text = '({}, {})'.format(x + w/2, y + h/2)
                cv2.putText(cur_frame, text, (x-50, y-5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

                self.cur_pos.data = [x + w/2, y + h/2]
                self.xy_pub.publish(self.cur_pos)

            # update fps counter
            self.fps.update()
            self.fps.stop()

            # initialize the set of information we'll be displaying on the frame
            info = [
                ('Tracker', 'csrt'),
                ('Success', "Yes" if success else "No"),
                ('FPS', "{:.2f}".format(self.fps.fps()))
            ]

            # loop the info and print on frame
            for (i, (k, v)) in enumerate(info):
                text = '{}: {}'.format(k, v)
                cv2.putText(cur_frame, text, (10, H - (i * 20 + 20)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        return cur_frame

    def KNN_update_csrt(self, current_frame_gray, cur_frame):
        morph_cur = self.GMM_Morph(current_frame_gray)

        contours, _ = cv2.findContours(morph_cur, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

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
                    dist = np.sqrt((x+w/2 - xr-wr/2) ** 2 + (y+h/2 - yr-hr/2) ** 2)
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
            if i+1 < n:
                for j in range(i+1, n):
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

        for x, y, w, h in contoursRect:
            arr = []
            arr.append((x, y))
            arr.append((x + w, y + h))
            x, y, w, h = cv2.boundingRect(np.asarray(arr))

            if w*h <= 3000:
                # cv2.fillPoly(morph_cur, pts=[np.asarray([(x, y), (x, y+h), (x+w, y+h), (x+w, y)])], color=0)
                # continue

                cv2.rectangle(cur_frame, (x, y), (x + w, y + h), (0, 0, 255), 1)  # bgr

                if self.target:
                    xt, yt, wt, ht = self.target
                    dist = np.sqrt((x + w / 2 - xt - wt / 2) ** 2 + (y + h / 2 - yt - ht / 2) ** 2)
                    if not self.initBB and dist < 60:
                        print("init initBB")
                        self.initBB = (x, y, (wt+w)/2, (ht+h)/2)
                        self.target = self.initBB
                        self.tracker.init(cur_frame, self.initBB)

                    elif self.initBB and 10 < dist < 15:
                        # update ROI of CSRT
                        print("update initBB size")
                        self.tracker = cv2.TrackerCSRT_create()
                        self.initBB = None
                        self.lastBB = None
                        self.fps = None
                        self.fps = FPS().start()
                        self.initBB = (0.25*x+0.75*xt, 0.25*y+0.75*yt, 0.75*wt + 0.25*w, 0.75*ht + 0.25*h)  # momentom
                        self.target = self.initBB
                        self.tracker.init(cur_frame, self.initBB)

            # box = cv2.minAreaRect(np.asarray(arr))
            # box = cv2.boxPoints(box)  # 4 outer corners
            # # convert all coordinates floating point values to int
            # box = np.int0(box)
            # cv2.drawContours(cur_frame, [box], 0, (0, 255, 0), 2)

        return cur_frame

    def GMM_Morph(self, cur_frame_gray):
        _, threshold_cur = cv2.threshold(cur_frame_gray, 30, 255, cv2.THRESH_BINARY)
        GMMED = self.fgbg.apply(threshold_cur)
        # mask = ROI(GMMED, 430, 330, 500, 300)
        gauss_blur = cv2.GaussianBlur(GMMED, (3, 3), 0)
        _, threshold_cur = cv2.threshold(gauss_blur, 30, 255, cv2.THRESH_BINARY)
        thresh_gray = cv2.morphologyEx(threshold_cur, cv2.MORPH_CLOSE,
                                       cv2.getStructuringElement(cv2.MORPH_DILATE, (17, 17)))

        thresh_gray = cv2.morphologyEx(thresh_gray, cv2.MORPH_CLOSE,
                                       cv2.getStructuringElement(cv2.MORPH_OPEN, (17, 17)))

        # masked = cv2.bitwise_and(thresh_gray, mask)
        contours, _ = cv2.findContours(thresh_gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # outlier rejection according to contour area, aspect ratio and amount.
        if len(contours) > 20:
            for c in contours:
                cv2.fillPoly(thresh_gray, pts=[c], color=0)

        else:
            for c in contours:
                area = cv2.contourArea(c)

                # Fill very small contours with zero (erase small contours).
                if area < 100 or area > 2500:
                    cv2.fillPoly(thresh_gray, pts=[c], color=0)
                    continue

            # Erase small contours, and contours which small aspect ratio (close to a square)
            # # https://stackoverflow.com/questions/52247821/find-width-and-height-of-rotatedrect
                rect = cv2.minAreaRect(c)
                (x, y), (w, h), angle = rect
                aspect_ratio = max(w, h) / min(w, h)

                if (aspect_ratio > 4):
                    cv2.fillPoly(thresh_gray, pts=[c], color=0)
                    continue

        dilated = cv2.dilate(thresh_gray, (3, 3), iterations=2)
        median_blur = cv2.medianBlur(dilated, 3)
        # # Use "close" morphological operation to close the gaps between contours
        # # https://stackoverflow.com/questions/18339988/implementing-imcloseim-se-in-opencv
        thresh_gray = cv2.morphologyEx(median_blur, cv2.MORPH_CLOSE,
                                       cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (1, 1)))

        return thresh_gray


    def ROI(self, img, x, y, w, h):
        vertices = [(x-w/2, y-h/2), (x-w/2, y+h/2), (x+w/2, y+h/2), (x+w/2, y-h/2), (x-w/2, y-h/2)]
        mask = np.zeros_like(img)
        # channel_count = image.shape[2]
        # match_mask_color = (255, ) * channel_count
        match_mask_color = 255
        cv2.fillPoly(mask, np.array([vertices], dtype=np.int32), match_mask_color)
        return mask


    def gray_contrast_brightness(self, cur_frame_gray, alpha, beta):
        new_image = cur_frame_gray.copy()
        m, n = new_image.shape
        new_image[:, :] = np.clip(np.ones([1, n]) * alpha * cur_frame_gray + np.ones([1, n])*beta, 0, 255)

        return new_image


    def imageCB(self, msg):
        current_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        current_frame = imutils.resize(current_frame, width=500)

        temp = current_frame.copy()
        current_frame_gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
        current_frame_gray = self.gray_contrast_brightness(current_frame_gray, 2, 0)

        morph_cur = self.GMM_Morph(current_frame_gray)
        KNN = self.KNN_update_csrt(current_frame_gray, temp)
        KNN_csrt = self.csrt_tracker(current_frame)

        cv2.imshow('Morph_Mask', morph_cur)
        cv2.imshow("KNN_csrt", KNN_csrt)  # KNN_csrt

        key = cv2.waitKey(1)
        if key & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            cap.release()
        if key == ord('p'):
            if not self.initBB:
                self.initBB = cv2.selectROI('KNN_csrt', KNN_csrt, fromCenter=False,
                                       showCrosshair=True)
                self.target = self.initBB
                # start OPenCV object tracker using the supplied bounding box
                # coordinate, then start FPS throughput estimator as well
                self.tracker.init(KNN_csrt, self.initBB)
                self.fps = FPS().start()
            else:
                cv2.waitKey(-1)  # wait until any key is pressed


detection = ultrasound_detection()
rospy.spin()
