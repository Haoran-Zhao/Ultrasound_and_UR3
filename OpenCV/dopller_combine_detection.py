import cv2
import numpy as np
import imutils
from imutils.video import VideoStream, FPS
import os.path
import csv

fgbg = cv2.bgsegm.createBackgroundSubtractorMOG()


def read_images(path, maxn):
    idx = 1
    images = []
    img_added = None
    mask = cv2.imread('Background/doppler_bg_mask2.png')
    mask = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)

    for i in range(idx, maxn+1):
        img_path = path + 'scene' + str(idx) + '.png'
        if os.path.isfile(img_path):
            img = cv2.imread(img_path)
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            img = imutils.resize(img, height=500)
            _, thre = cv2.threshold(img, 40, 255, cv2.THRESH_BINARY)
            masked = cv2.bitwise_and(thre, mask)
            cv2.imshow('img', masked)
            cv2.waitKey(20)
            images.append(img)
            if img_added is None:
                img_added = thre.copy()
            else:
                img_added = cv2.add(img_added, thre)
            idx = idx+1
        else:
            idx = idx +1

    return images, img_added


def sum_bg():
    path = 'Data/dopller_combine/'
    img_set, img_added = read_images(path, 3912)
    # _, img_added = cv2.threshold(img_added, 60, 255, cv2.THRESH_BINARY)
    cv2.imshow('added', img_added)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    cv2.imwrite('/Background/doppler_combine_bg.png', img_added)
    return img_set, img_added


def find_mask():
    img = cv2.imread('Background/doppler_combine_bg.png')
    flipped = cv2.flip(img, 1)
    added = cv2.add(img, flipped)
    cv2.imshow('added', added)
    cv2.imwrite('Background/doppler_flipped_bg.png', added)

    # added = cv2.dilate(added, (33,33), iterations=8)
    imgray = cv2.cvtColor(added, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(imgray, 50, 255, 0)
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours_sizes = [(cv2.contourArea(cnt), cnt) for cnt in contours]
    primer = max(contours_sizes, key=lambda x: x[0])[1]

    epsilon = 0.005 * cv2.arcLength(primer, True)
    approx = cv2.approxPolyDP(primer, epsilon, True)
    cv2.drawContours(added, [approx], -1, (0, 255, 0), 3)
    # cv2.drawContours(added, [biggest_contour], 0, (0, 255, 0), 3)
    # added = cv2.erode(added, (3,3), iterations=3)
    # cv2.imshow('origin', img)
    cv2.imshow('processed', added)
    # cv2.imwrite('BG_ROI_added.png', added)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def GMM_Morph(cur_frame, mask, threshold):
    cur_gray = cv2.cvtColor(cur_frame, cv2.COLOR_BGR2GRAY)
    masked = cv2.bitwise_and(cur_gray, mask)
    cur_thre = cv2.adaptiveThreshold(masked, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 5, 1)
    thre_GMM = fgbg.apply(cur_thre)
    gauss_blur = cv2.GaussianBlur(thre_GMM, (3, 3), 0)
    _, threshold_cur = cv2.threshold(gauss_blur, 30, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(threshold_cur, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for c in contours:
        area = cv2.contourArea(c)

        # Fill very small contours with zero (erase small contours).
        if area < 30 or area > 2500:
            cv2.fillPoly(threshold_cur, pts=[c], color=0)
            continue

        # Erase small contours, and contours which small aspect ratio (close to a square)
        # # https://stackoverflow.com/questions/52247821/find-width-and-height-of-rotatedrect
        rect = cv2.minAreaRect(c)
        (x, y), (w, h), angle = rect
        aspect_ratio = max(w, h) / min(w, h)

        if (aspect_ratio < 2 or aspect_ratio>5):
            cv2.fillPoly(threshold_cur, pts=[c], color=0)
            continue
    thresh_gray = cv2.morphologyEx(threshold_cur, cv2.MORPH_CLOSE,
                                   cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7)))
    eroded = cv2.erode(thresh_gray, (3, 3), iterations=2)
    # median_blur = cv2.medianBlur(dilated, 3)

    return eroded

def find_contour(path, maxn):
    idx = 1
    images = []
    cords = []
    img_added = None
    mask = cv2.imread('Background/doppler_bg_mask2.png')
    mask = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
    _, mask = cv2.threshold(mask, 40, 255, cv2.THRESH_BINARY)
    # cv2.imshow('mask', mask)

    for i in range(idx, maxn + 1):
        img_path = path + 'scene' + str(i) + '.png'
        if os.path.isfile(img_path):
            img = cv2.imread(img_path)
            temp = img.copy()
            img = imutils.resize(img, height=500)
            GMMED = GMM_Morph(img, mask, 20)
            cv2.imshow('GMMED', GMMED)

            contours, hierarchy = cv2.findContours(GMMED, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
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
                    cords.append([x + w / 2, y + h / 2])
                    cv2.putText(temp, text, (x - 50, y - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                    # cv2.drawContours(temp, [box], 0, (0, 255, 0), 2)
            cv2.imshow('img', temp)
            cv2.waitKey(20)
    with open('GFG', 'w') as f:
        write = csv.writer(f)

        write.writerows(cords)



path = 'Data/dopller_combine/'
# img_set, img_added = sum_bg()
# find_mask()
find_contour(path, 3912)
