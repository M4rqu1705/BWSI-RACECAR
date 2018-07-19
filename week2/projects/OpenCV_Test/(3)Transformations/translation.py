import cv2
import numpy as np

# Read doge image with full color
original = cv2.imread('/home/racecar/Documents/OpenCV_Test/doge.jpg')

# Extract the width and height of the image
width, height = original.shape[:2]

# Establish units of translation in X and Y axis
x_translation, y_translation = 150, 100
# Basically have an identity matrix with other values to the right
translation_matrix = np.float32([[1, 0, x_translation], [0, 1, y_translation]])

# Warp affine with image to transform, matrix and image dimensions
translated = cv2.warpAffine(original, translation_matrix, (height, width))

# Show the results of the translation
cv2.namedWindow('Doge', cv2.WINDOW_NORMAL)
cv2.imshow('Doge', original)
cv2.namedWindow('Rotated Doge', cv2.WINDOW_NORMAL)
cv2.imshow('Rotated Doge', translated)

# Wait until the user presses any key to continue to the next part
cv2.waitKey(0)

# Destroy all windows containing images
cv2.destroyAllWindows()
