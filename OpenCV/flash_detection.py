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
    mask = cv2.imread('Background/flash_bg_mask.png')
    mask = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)

    for i in range(idx, maxn+1):
        img_path = path + 'scene' + str(idx) + '.png'
        print(i)
        if os.path.isfile(img_path):
            img = cv2.imread(img_path)
            temp = img.copy()
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            img = imutils.resize(img, height=500)
            img = cv2.bitwise_and(img, mask)
            img = preprocess(img)
            cv2.imshow('img', img)
            contoursRect = find_contour(img)
            for x, y, w, h in contoursRect:
                arr = []
                arr.append((x, y))
                arr.append((x + w, y + h))
                x, y, w, h = cv2.boundingRect(np.asarray(arr))
                if w * h <= 3000:
                    cv2.rectangle(temp, (x, y), (x + w, y + h), (0, 0, 255), 1)
            cv2.imwrite('detection_result/detect' + str(i) + '.png', temp)
            cv2.imshow('cnt', temp)
            cv2.waitKey(20)
            idx = idx + 1
        else:
            idx = idx + 1

    return images


def preprocess(img, alpha=2, beta=-50):
    img = contrast_brightness(img, alpha, beta)
    _, img = cv2.threshold(img, 30, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for c in contours:
        area = cv2.contourArea(c)

        # Fill very small contours with zero (erase small contours).
        if area < 30 or area > 2500:
            cv2.fillPoly(img, pts=[c], color=0)
            continue

        # Erase small contours, and contours which small aspect ratio (close to a square)
        # # https://stackoverflow.com/questions/52247821/find-width-and-height-of-rotatedrect
        # rect = cv2.minAreaRect(c)
        # (x, y), (w, h), angle = rect
        # aspect_ratio = max(w, h) / min(w, h)
    #
    #     if (aspect_ratio < 2 or aspect_ratio > 5):
    #         cv2.fillPoly(img, pts=[c], color=0)
    #         continue
    #
    return img

def find_contour(img):
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

def contrast_brightness(cur_frame_gray, alpha, beta):
    new_image = cur_frame_gray.copy()
    m, n = new_image.shape
    new_image[:, :] = np.clip(np.ones([1, n]) * alpha * cur_frame_gray + np.ones([1, n])*beta, 0, 255)

    return new_image


def sum_bg():
    path = 'Data/flash/'
    img_set, img_added = read_images(path, 4312)
    # _, img_added = cv2.threshold(img_added, 60, 255, cv2.THRESH_BINARY)
    cv2.imshow('added', img_added)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    cv2.imwrite('Background/Flash_bg.png', img_added)
    return img_set, img_added


def find_mask():
    img = cv2.imread('Background/Flash_bg.png')
    flipped = cv2.flip(img, 1)
    added = cv2.add(img, flipped)
    cv2.imshow('added', added)
    cv2.imwrite('Background/flash_flipped_bg.png', added)

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
    added = cv2.erode(added, (3,3), iterations=3)
    # cv2.imshow('origin', img)
    cv2.imshow('processed', added)
    # cv2.imwrite('BG_ROI_added.png', added)
    cv2.waitKey(0)
    cv2.destroyAllWindows()



path = 'Data/flash/'
read_images(path, 4312)
# img_set, img_added = sum_bg()
# find_mask()
# find_contour(path, 4312)
