import cv2
import numpy as np

# Read doge image with full color
original = cv2.imread('/home/racecar/Documents/OpenCV_Test/doge.jpg')

# Test different methods of scaling
scaled_area = cv2.resize(original, None, fx=0.05, fy=0.05, interpolation = cv2.INTER_AREA)
scaled_linear = cv2.resize(original, None, fx=0.05, fy=0.05, interpolation = cv2.INTER_LINEAR)
scaled_cubic = cv2.resize(original, None, fx=0.05, fy=0.05, interpolation = cv2.INTER_CUBIC)

# Show the different results of each of the methods of scaling
cv2.namedWindow('Doge', cv2.WINDOW_NORMAL)
cv2.imshow('Doge', original)
cv2.namedWindow('Scaled Area', cv2.WINDOW_NORMAL)
cv2.imshow('Scaled Area', scaled_area)
cv2.namedWindow('Scaled Linear', cv2.WINDOW_NORMAL)
cv2.imshow('Scaled Linear', scaled_linear)
cv2.namedWindow('Scaled Cubic', cv2.WINDOW_NORMAL)
cv2.imshow('Scaled Cubic', scaled_cubic)

# Wait until the user presses any key to continue to the next part
cv2.waitKey(0)

# Destroy all windows containing images
cv2.destroyAllWindows()
