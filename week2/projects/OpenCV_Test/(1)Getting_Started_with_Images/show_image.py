import numpy as np
import cv2

img = cv2.imread('/home/racecar/Documents/OpenCV_Test/doge.jpg', 0)
cv2.namedWindow('doge.jpg', cv2.WINDOW_NORMAL)
cv2.imshow('doge.jpg', img)
cv2.waitKey(0)
cv2.destroyAllWindows()
