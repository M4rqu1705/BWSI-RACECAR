import cv2
import numpy as np

img = cv2.imread('/home/racecar/Documents/OpenCV_Test/doge.jpg')

# Convert BGR to HSV
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

# Define range of orange color in HSV
lower_red = np.array([0,200,200])
upper_red = np.array([255,255,255])

# Threshold the HSV image to get only red colors
mask = cv2.inRange(hsv, lower_red, upper_red)

# Bitwise-AND mask and original image
res = cv2.bitwise_and(img,img, mask= mask)
cv2.namedWindow('Original', cv2.WINDOW_NORMAL)
cv2.namedWindow('Mask', cv2.WINDOW_NORMAL)
cv2.namedWindow('Res', cv2.WINDOW_NORMAL)
cv2.imshow('Original', img)
cv2.imshow('Mask',mask)
cv2.imshow('Res',res)
cv2.waitKey(0)

cv2.destroyAllWindows()
