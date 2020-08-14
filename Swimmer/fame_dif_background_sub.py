import cv2
import numpy as np

cap = cv2.VideoCapture('Data/30HzTest.avi')
fgbg = cv2.bgsegm.createBackgroundSubtractorMOG()
fgbg2 = cv2.bgsegm.createBackgroundSubtractorMOG()

ret, current_frame = cap.read()
previous_frame = current_frame


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
        cv2.imshow('frame diff thre', frame_diff_thres)


def prev_cur_gray(cur_frame, prev_frame):
    current_frame_gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
    previous_frame_gray = cv2.cvtColor(previous_frame, cv2.COLOR_BGR2GRAY)
    return current_frame_gray, previous_frame_gray


def diff_GMM_Morph(frame):
    gaus_blurred = cv2.GaussianBlur(frame, (5, 5), 0)  # low pass filter
    median_blurred = cv2.medianBlur(gaus_blurred, 5)
    GMMed = fgbg.apply(median_blurred)  # apply GMM for background sub
    # kernel1 = np.ones((3, 3), np.uint8)
    # closing = cv2.morphologyEx(GMMed, cv2.MORPH_CLOSE, kernel1, iterations=3)
    kernel2 = np.ones((1, 1), np.uint8)
    result = cv2.morphologyEx(GMMed, cv2.MORPH_OPEN, kernel2, iterations=3)  # erosion followed by dilation
    return result

def ROI(img, x, y, w, h):
    vertices = [(x-w/2, y-h/2), (x-w/2, y+h/2), (x+w/2, y+h/2), (x+w/2, y-h/2), (x-w/2, y-h/2)]
    mask = np.zeros_like(img)
    # channel_count = image.shape[2]
    # match_mask_color = (255, ) * channel_count
    match_mask_color = 255
    cv2.fillPoly(mask, np.array([vertices], dtype=np.int32), match_mask_color)
    return mask


def GMM_Morph(cur_frame, frame_diff):
    _, thre = cv2.threshold(cur_frame, 30, 255, cv2.THRESH_BINARY)
    gaus_cur = cv2.GaussianBlur(thre, (11, 11), 0)  # low pass filter
    gaus_diff = cv2.GaussianBlur(frame_diff, (11, 11), 0)  # low pass filter
    added = cv2.bitwise_and(gaus_cur, gaus_diff)
    GMMed = fgbg2.apply(added)  # apply GMM for background sub
    median_blurred = cv2.medianBlur(GMMed, 11)
    # kernel1 = np.ones((3, 3), np.uint8)
    # closing = cv2.morphologyEx(GMMed, cv2.MORPH_CLOSE, kernel1, iterations=3)
    # kernel2 = np.ones((1, 1), np.uint8)
    # result = cv2.morphologyEx(GMMed, cv2.MORPH_OPEN, kernel2, iterations=3)  # erosion followed by dilation
    return median_blurred


def GMM_Morph_ROI(cur_frame, frame_diff):
    mask = ROI(frame_diff, 400, 220, 110, 80)
    _, thre = cv2.threshold(cur_frame, 30, 255, cv2.THRESH_BINARY)
    gaus_cur = cv2.GaussianBlur(thre, (11, 11), 0)  # low pass filter
    gaus_diff = cv2.GaussianBlur(frame_diff, (11, 11), 0)  # low pass filter
    added = cv2.bitwise_and(gaus_cur, gaus_diff)
    GMMed = fgbg2.apply(added)  # apply GMM for background sub
    median_blurred = cv2.medianBlur(GMMed, 11)
    masked = cv2.bitwise_and(median_blurred, mask)
    # kernel1 = np.ones((3, 3), np.uint8)
    # closing = cv2.morphologyEx(GMMed, cv2.MORPH_CLOSE, kernel1, iterations=3)
    # kernel2 = np.ones((1, 1), np.uint8)
    # result = cv2.morphologyEx(GMMed, cv2.MORPH_OPEN, kernel2, iterations=3)  # erosion followed by dilation
    return masked


def frame_differencing(cur_frame, prev_frame):
    result = cv2.absdiff(cur_frame,  prev_frame)
    return result


def frame_diff_thre(cur_frame, prev_frame, threshold1, threshold2):
    _, threshold_cur = cv2.threshold(cur_frame, threshold1, 255, cv2.THRESH_BINARY)
    _, threshold_pre = cv2.threshold(prev_frame, threshold2, 255, cv2.THRESH_BINARY)
    result = cv2.absdiff(threshold_cur, threshold_pre)
    return result


def threshold_Morph(frame, threshold):
    _, threshold = cv2.threshold(frame, threshold, 255, cv2.THRESH_BINARY)
    kernel = np.ones((5, 5), np.uint8)
    result = cv2.morphologyEx(threshold, cv2.MORPH_CLOSE, kernel)  # erosion followed by dilatio
    return result


def nothing(x):
    pass


cv2.namedWindow('frame diff thre')
cv2.createTrackbar('thre1', 'frame diff thre', 30, 255, nothing)
cv2.createTrackbar('thre2', 'frame diff thre', 50, 255, nothing)

cv2.namedWindow('cur frame')
switch = 'Contour ON\OFF'
cv2.createTrackbar(switch, 'cur frame', 0, 1, nothing)
switch2 = 'Mask ON\OFF'
cv2.createTrackbar(switch2, 'cur frame', 0, 1, nothing)
count = 0

while cap.isOpened():
    if not ret:
        break

    thre1 = cv2.getTrackbarPos('thre1', 'frame diff thre')
    thre2 = cv2.getTrackbarPos('thre2', 'frame diff thre')
    s = cv2.getTrackbarPos(switch, 'cur frame')
    ms = cv2.getTrackbarPos(switch2, 'cur frame')

    temp = current_frame.copy()
    current_frame_gray, previous_frame_gray = prev_cur_gray(current_frame, previous_frame)
    frame_diff = frame_differencing(current_frame_gray, previous_frame_gray)
    frame_diff1 = frame_diff_thre(current_frame, previous_frame, 20, 20)

    frame_diff_thres = frame_diff_thre(current_frame, previous_frame, thre1, thre2)
    frame_diff_gaus = cv2.GaussianBlur(frame_diff_thres, (5, 5), 0)

    bg_sub = fgbg.apply(frame_diff_gaus)

    morph = diff_GMM_Morph(frame_diff_thres)

    if ms:
        morph_cur = GMM_Morph_ROI(current_frame_gray, cv2.cvtColor(frame_diff_thres, cv2.COLOR_RGB2GRAY))
    else:
        morph_cur = GMM_Morph(current_frame_gray, cv2.cvtColor(frame_diff_thres, cv2.COLOR_RGB2GRAY))

    contours, _ = cv2.findContours(morph_cur, cv2.RETR_EXTERNAL, 2)

    if s:
        for cnt in contours:
            (x, y), radius = cv2.minEnclosingCircle(cnt)
            center = (int(x), int(y))
            radius = int(radius)
            if 3 <= radius <= 15:
                cv2.circle(temp, center, radius, (0, 255, 0), 2)

    # path = 'C:/Users/zhaoh/PycharmProjects/Opencv/Swimmer/Frame differnce/41hz/image{}.png'.format(count)
    # cv2.imwrite(path, frame_diff)
    # count += 1

    cv2.imshow('cur frame', temp)
    cv2.imshow('frame diff', frame_diff)
    cv2.imshow('frame diff thre', frame_diff_thres)
    # cv2.imshow('frame diff gaus', frame_diff_gaus)
    # cv2.imshow('bg sub', bg_sub)
    cv2.imshow('morphy', morph_cur)

    cv2.setMouseCallback('frame diff thre', click_event)

    key = cv2.waitKey(1)
    if key & 0xFF == ord('q'):
        break
    if key == ord('p'):
        cv2.waitKey(-1)  # wait until any key is pressed

    previous_frame = current_frame.copy()
    ret, current_frame = cap.read()

cv2.destroyAllWindows()
cap.release()
