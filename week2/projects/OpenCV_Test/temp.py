#/usr/bin/env python
import cv2
import numpy as np

cv2.namedWindow('Video', cv2.WINDOW_NORMAL)
cv2.namedWindow('Blur', cv2.WINDOW_NORMAL)
cv2.namedWindow('Trackbar', cv2.WINDOW_NORMAL)

cap = cv2.VideoCapture(0)
cv2.createTrackbar('Min Val: ', 'Trackbar', 0, 1000, lambda *args: None)
cv2.createTrackbar('Max Val: ', 'Trackbar', 1000, 1000, lambda *args: None)
cv2.createTrackbar('Blur: ', 'Trackbar', 1, 10, lambda *args: None)

while True:
    _, frame = cap.read()    
    #  gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    #  blur = cv2.GaussianBlur(frame,(15, 15),0)
    blur = cv2.medianBlur(frame, cv2.getTrackbarPos('Blur: ', 'Trackbar')*2+1)


    edges = cv2.Canny(blur,cv2.getTrackbarPos('Min Val: ', 'Trackbar'), cv2.getTrackbarPos('Max Val: ', 'Trackbar'))

    cv2.imshow('Video', edges)
    cv2.imshow('Blur', blur)

    cv2.waitKey(1)

cv2.destroyAllWindows()
