import cv2
import numpy as np

# Read doge image with full color
original = cv2.imread('/home/racecar/Documents/OpenCV_Test/doge.jpg')

# Test different methods of scaling
width, height, ch = original.shape

# Prepare original points for Perspective transformation
pts1 = np.float32([[200, 200], [width-200, 200], [0, height], [width, height]])

# Prepare future position of corresponding points after Perspective transformation
pts2 = np.float32([[0, 0], [width, 0], [0, height], [width, height]])

# Generate transformation matrix
perspective_matrix = cv2.getPerspectiveTransform(pts1, pts2)

# Transform the original image
changed_perspective = cv2.warpPerspective(original, perspective_matrix, (height, width))

# Show the results of the transformation
cv2.namedWindow('Doge', cv2.WINDOW_NORMAL)
cv2.imshow('Doge', original)
cv2.namedWindow('Perspective', cv2.WINDOW_NORMAL)
cv2.imshow('Perspective', changed_perspective)

# Wait until the user presses any key to continue to the next part
cv2.waitKey(0)

# Destroy all windows containing images
cv2.destroyAllWindows()
