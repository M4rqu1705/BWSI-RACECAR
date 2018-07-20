import cv2
import numpy as np

cap = cv2.VideoCapture(0)

    
cv2.namedWindow('Video', cv2.WINDOW_NORMAL)
cv2.namedWindow('Mask', cv2.WINDOW_NORMAL)
cv2.namedWindow('Trackbar', cv2.WINDOW_NORMAL)

cv2.createTrackbar('Lower Hue: ', 'Trackbar', 0, 359, lambda *args : None)
cv2.createTrackbar('Higher Hue: ', 'Trackbar', 359, 359, lambda *args : None)
cv2.createTrackbar('Lower Saturation: ', 'Trackbar', 0, 255, lambda *args : None)
cv2.createTrackbar('Higher Saturation: ', 'Trackbar', 255, 255, lambda *args : None)
cv2.createTrackbar('Lower Value: ', 'Trackbar', 0, 255, lambda *args : None)
cv2.createTrackbar('Higher Value: ', 'Trackbar', 255, 255, lambda *args : None)

while True:
    _, frame = cap.read()
    frame = cv2.medianBlur(frame, 5)
    hsv_color_lower = np.array([cv2.getTrackbarPos('Lower Hue: ', 'Trackbar'), cv2.getTrackbarPos('Lower Saturation: ', 'Trackbar'), cv2.getTrackbarPos('Lower Value: ', 'Trackbar')])
    hsv_color_higher = np.array([cv2.getTrackbarPos('Higher Hue: ', 'Trackbar'), cv2.getTrackbarPos('Higher Saturation: ', 'Trackbar'), cv2.getTrackbarPos('Higher Value: ', 'Trackbar')])

    mask = cv2.inRange(frame, hsv_color_lower, hsv_color_higher)
    res = cv2.bitwise_and(frame, frame, mask = mask)
    

    cv2.imshow('Video', res)
    cv2.imshow('Mask', mask)
    cv2.waitKey(1)

cv2.destroyAllWindows()
