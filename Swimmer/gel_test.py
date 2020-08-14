import cv2
import numpy as np
import imutils
from imutils.video import VideoStream, FPS

# # extract the openCV version info
# (major, minor) = cv2.__version__.split('.')[:2]
# # if we using OpenCV 3.2 or before, we can use a special factory
# # function to create our object tracker
# if int(major) == 3 and int(minor) < 3:
#     tracker = cv2.Tracker_creat('csrt'.upper())
#
# else:
#     # initialize the dictionary of tracker
#     OPENCV_OBJECT_TRACKERS ={
#         'csrt': cv2.TrackerCSRT_create,
#         'kcf': cv2.TrackerKCF_create,
#         'boosting': cv2.TrackerBoosting_create,
#         'mil': cv2.TrackerMIL_create,
#         'tld': cv2.TrackerTLD_create,
#         'medianflow': cv2.TrackerMedianFlow_create,
#         'mosse': cv2.TrackerMOSSE_create
#     }


def csrt_tracker(cur_frame):
    global initBB
    global lastBB
    global fps
    global tracker
    global target

    (H, W) = cur_frame.shape[:2]

    # check if we are currently tacking an object
    if initBB is not None:
        # grab new bounding box coordinates of the object
        (success, box) = tracker.update(cur_frame)

        # check to see if the tracking was success
        if success:
            if lastBB:
                (xl, yl, wl, hl) = [int(v) for v in lastBB]
                (x, y, w, h) = [int(v) for v in box]

                dist = np.sqrt((x+w/2 - xl-wl/2) ** 2 + (y+h/2 - yl-hl/2) ** 2)
                if dist > 40:
                    print(dist)
                    tracker = cv2.TrackerCSRT_create()
                    initBB = None
                    lastBB = None
                    fps = None
                    fps = FPS().start()
                    return cur_frame

                else:
                    target = (x, y, w, h)
                    cv2.rectangle(cur_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            else:
                (x, y, w, h) = [int(v) for v in box]
                cv2.rectangle(cur_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

            lastBB = box


        # update fps counter
        fps.update()
        fps.stop()

        # initialize the set of information we'll be displaying on the frame
        info = [
            ('Tracker', 'csrt'),
            ('Success', "Yes" if success else "No"),
            ('FPS', "{:.2f}".format(fps.fps()))
        ]

        # loop the info and print on frame
        for (i, (k, v)) in enumerate(info):
            text = '{}: {}'.format(k, v)
            cv2.putText(cur_frame, text, (10, H - (i * 20 + 20)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    return cur_frame


def click_event(event, x, y, flag, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        print(x, ', ', y)
        font = cv2.FONT_HERSHEY_SIMPLEX
        strXY = str(x) + ', ' + str(y)
        cv2.putText(frame_diff_thres, strXY, (x, y), font, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
        cv2.circle(frame_diff_thres, (x, y), 3, (0, 0, 255), -1)

    if event == cv2.EVENT_RBUTTONDOWN:
        gray = frame_diff_thres[y, x]
        print(gray)
        cv2.circle(frame_diff_thres, (x, y), 3, (0, 0, 255), -1)

        font = cv2.FONT_HERSHEY_SIMPLEX
        strBGR = str(gray)
        cv2.putText(frame_diff_thres, strBGR, (x, y), font, 0.5, (0, 0, 255),
                    1, cv2.LINE_AA)
        cv2.imshow('morphy', frame_diff_thres)


def prev_cur_gray(cur_frame, prev_frame):
    current_frame_gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
    previous_frame_gray = cv2.cvtColor(previous_frame, cv2.COLOR_BGR2GRAY)
    return current_frame_gray, previous_frame_gray


def ROI(img, x, y, w, h):
    vertices = [(x-w/2, y-h/2), (x-w/2, y+h/2), (x+w/2, y+h/2), (x+w/2, y-h/2), (x-w/2, y-h/2)]
    mask = np.zeros_like(img)
    # channel_count = image.shape[2]
    # match_mask_color = (255, ) * channel_count
    match_mask_color = 255
    cv2.fillPoly(mask, np.array([vertices], dtype=np.int32), match_mask_color)
    return mask


def customed_pro(cur_frame_gray, cur_frame):
    global target
    global initBB
    global lastBB
    global tracker
    global fps
    morph_cur = GMM_Morph(current_frame_gray)

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
            for i, dims in enumerate(contoursRect):
                [xr, yr, wr, hr] = dims
                dist = np.sqrt((x+w/2 - xr-wr/2) ** 2 + (y+h/2 - yr-hr/2) ** 2)
                if dist < mind and dist < 50:
                    mind = dist
                    index = i

            if index == -1:
                contoursRect.append((x, y, w, h))
            else:
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

            cv2.rectangle(cur_frame, (x, y), (x + w, y + h), (0, 0, 255), 1)

            if target:
                xt, yt, wt, ht = target
                dist = np.sqrt((x + w / 2 - xt - wt / 2) ** 2 + (y + h / 2 - yt - ht / 2) ** 2)
                if not initBB and dist < 60:
                    initBB = (x, y, (wt+w)/2, (ht+h)/2)
                    target = initBB
                    tracker.init(cur_frame, initBB)

                elif initBB and 10 < dist < 15:
                    tracker = cv2.TrackerCSRT_create()
                    initBB = None
                    lastBB = None
                    fps = None
                    fps = FPS().start()
                    initBB = (0.25*x+0.75*xt, 0.25*y+0.75*yt, 0.75*wt + 0.25*w, 0.75*ht + 0.25*h)
                    target = initBB
                    tracker.init(cur_frame, initBB)

        # box = cv2.minAreaRect(np.asarray(arr))
        # box = cv2.boxPoints(box)  # 4 outer corners
        # # convert all coordinates floating point values to int
        # box = np.int0(box)
        # cv2.drawContours(cur_frame, [box], 0, (0, 255, 0), 2)

    return cur_frame


def GMM_Morph(cur_frame):
    _, threshold_cur = cv2.threshold(cur_frame, 30, 255, cv2.THRESH_BINARY)
    GMMED = fgbg.apply(threshold_cur)
    # mask = ROI(GMMED, 430, 330, 500, 300)
    gauss_blur = cv2.GaussianBlur(GMMED, (3, 3), 0)
    _, threshold_cur = cv2.threshold(gauss_blur, 30, 255, cv2.THRESH_BINARY)
    thresh_gray = cv2.morphologyEx(threshold_cur, cv2.MORPH_CLOSE,
                                   cv2.getStructuringElement(cv2.MORPH_DILATE, (17, 17)))

    thresh_gray = cv2.morphologyEx(thresh_gray, cv2.MORPH_CLOSE,
                                   cv2.getStructuringElement(cv2.MORPH_OPEN, (17, 17)))

    # masked = cv2.bitwise_and(thresh_gray, mask)
    contours, _ = cv2.findContours(thresh_gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

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


def frame_differencing(cur_frame, prev_frame):
    result = cv2.absdiff(cur_frame,  prev_frame)
    return result


def frame_diff_thre(cur_frame, prev_frame, threshold1, threshold2):
    _, threshold_cur = cv2.threshold(cur_frame, threshold1, 255, cv2.THRESH_BINARY)
    _, threshold_pre = cv2.threshold(prev_frame, threshold2, 255, cv2.THRESH_BINARY)
    result = cv2.absdiff(threshold_cur, threshold_pre)
    return result

def nothing(x):
    pass


cap = cv2.VideoCapture('Data/gel.avi')
fgbg = cv2.bgsegm.createBackgroundSubtractorMOG()
fgbg2 = cv2.bgsegm.createBackgroundSubtractorMOG()

ret, current_frame = cap.read()
current_frame = imutils.resize(current_frame, width=500)
previous_frame = current_frame

tracker = cv2.TrackerCSRT_create()

# initialize the bounding box coordinate of the object we are going to track
initBB = None
lastBB = None
fps = None
target = None
# cv2.namedWindow('frame diff thre')
# cv2.createTrackbar('thre1', 'frame diff thre', 26, 255, nothing)
# cv2.createTrackbar('thre2', 'frame diff thre', 26, 255, nothing)

# cv2.namedWindow('cur frame')
# switch = 'Contour ON\OFF'
# cv2.createTrackbar(switch, 'cur frame', 1, 1, nothing)
count = 0

while cap.isOpened():
    if not ret:
        break

    thre1 = cv2.getTrackbarPos('thre1', 'frame diff thre')
    thre2 = cv2.getTrackbarPos('thre2', 'frame diff thre')
    # s = cv2.getTrackbarPos(switch, 'cur frame')
    s = 1

    temp = current_frame.copy()
    temp2 = current_frame.copy()
    current_frame_gray, previous_frame_gray = prev_cur_gray(current_frame, previous_frame)
    # frame_diff = frame_differencing(current_frame_gray, previous_frame_gray)
    # frame_diff1 = frame_diff_thre(current_frame, previous_frame, 20, 20)

    frame_diff_thres = frame_diff_thre(current_frame, previous_frame, thre1, thre2)


    morph_cur = GMM_Morph(current_frame_gray)

    # Find contours in thresh_gray after closing the gaps
    contours, _ = cv2.findContours(morph_cur, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    #
    # if len(contours) > 20:
    #     contours = []

    if s:
        for c in contours:
            area = cv2.contourArea(c)

            # Small contours are ignored.
            # if area < 500:
            #     cv2.fillPoly(thresh_gray, pts=[c], color=0)
            #     continue

            rect = cv2.minAreaRect(c)
            box = cv2.boxPoints(rect)
            # convert all coordinates floating point values to int
            box = np.int0(box)
            cv2.drawContours(temp, [box], 0, (0, 255, 0), 2)

    new_method = customed_pro(current_frame_gray, temp2)
    csrt = csrt_tracker(current_frame)
    # if s:
    #     for cnt in contours:
    #         (x, y), radius = cv2.minEnclosingCircle(cnt)
    #         center = (int(x), int(y))
    #         radius = int(radius)
    #         if 3 <= radius <= 15:
    #             cv2.circle(temp, center, radius, (0, 255, 0), 2)

    # path = 'C:/Users/zhaoh/PycharmProjects/Opencv/Swimmer/Frame differnce/41hz/image{}.png'.format(count)
    # cv2.imwrite(path, frame_diff)
    # count += 1

    cv2.imshow('cur frame', temp)
    cv2.imshow('new', new_method)
    cv2.imshow("csrt", csrt)
    # cv2.imshow('frame diff thre', frame_diff_thres)
    cv2.imshow('morphy', morph_cur)

    cv2.setMouseCallback('new', click_event)

    key = cv2.waitKey(20)
    if key & 0xFF == ord('q'):
        break
    if key == ord('p'):
        if not initBB:
            initBB = cv2.selectROI('csrt', csrt, fromCenter=False,
                                   showCrosshair=True)
            target = initBB
            # start OPenCV object tracker using the supplied bounding box
            # coordinate, then start FPS throughput estimator as well
            tracker.init(csrt, initBB)
            fps = FPS().start()
        else:
            cv2.waitKey(-1)  # wait until any key is pressed

    previous_frame = current_frame.copy()
    ret, current_frame = cap.read()
    current_frame = imutils.resize(current_frame, width=500)


cv2.destroyAllWindows()
cap.release()
