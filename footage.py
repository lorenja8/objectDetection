import time

import cv2 as cv
import numpy as np

from object_detection import detector
from tracker import tracker

cap = cv.VideoCapture(0, cv.CAP_DSHOW)
cap.set(cv.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv.CAP_PROP_FRAME_HEIGHT, 360)

# path = 'DSCF4447.JPG'
# obrazok = cv.imread(path)
# n = 0.08
# obrazok = cv.resize(obrazok, (0,0), fx=n, fy=n)

detect = detector()
track = tracker()

def empty(a):
    pass

cv.namedWindow("trackbar")
cv.resizeWindow("trackbar",640,360)
cv.createTrackbar("a", "trackbar", 0, 255, empty)
cv.createTrackbar("b", "trackbar", 255, 255, empty)
cv.createTrackbar("c", "trackbar", 0, 255, empty)
cv.createTrackbar("d", "trackbar", 255, 255, empty)

prev_frame_time = 0
new_frame_time = 0

blank = np.zeros((360, 640, 3), np.uint8)

def circle(image, placement):
    pass

cv.circle(blank, (50,50), 20, (255,255,255), cv.FILLED)

while True:
    cv.imshow("blank", blank)

    success, frame = cap.read()
    new_frame_time = time.time()
    fps = 1 / (new_frame_time - prev_frame_time)
    prev_frame_time = new_frame_time

    cv.putText(frame, str(int(fps)), (7, 70), cv.FONT_HERSHEY_SIMPLEX, 3, (100, 255, 0), 3, cv.LINE_AA)

    # frame = obrazok.copy()
    a = cv.getTrackbarPos("a", "trackbar")
    b = cv.getTrackbarPos("b", "trackbar")
    c = cv.getTrackbarPos("c", "trackbar")
    d = cv.getTrackbarPos("d", "trackbar")
    detect.parameters = (a, b, c, d)
    # gray, threshold_bw = detect.method_1(frame)
    whitefilter = detect.method_4(frame)

    detect.detect_contours(whitefilter)

    track.update(detect.ellipses)
    track.draw_ellipse(frame)

    # cv.imshow("gray", gray)
    # cv.imshow("thresh", threshold_bw)
    cv.imshow("whitefilter", whitefilter)
    cv.imshow("frame", frame)
    key = cv.waitKey(1) & 0xFF