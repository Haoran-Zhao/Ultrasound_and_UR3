import cv2
import numpy as np

cap = cv2.VideoCapture('Data/41HzTest.avi')
fgbg = cv2.bgsegm.createBackgroundSubtractorMOG()

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break
    fgmask = fgbg.apply(frame)

    cv2.imshow('Video', frame)
    cv2.imshow('FG Mask', fgmask)

    if cv2.waitKey(30) == 27:
        break

cv2.destroyAllWindows()
cap.release()