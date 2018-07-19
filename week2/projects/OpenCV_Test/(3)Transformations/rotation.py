import cv2
import numpy as np

# Read doge image with full color
original = cv2.imread('/home/racecar/Documents/OpenCV_Test/doge.jpg')

# Extract the width and height of the image
width, height = original.shape[:2]

# Create rotation matrix that will describe behavior of rotation
center_y, center_x, angle_rotation, scale = height/2, width/2, 45, 1
rotation_matrix = cv2.getRotationMatrix2D((center_y, center_x), angle_rotation, scale)

# Perform the transformation
rotated = cv2.warpAffine(original, rotation_matrix, (height, width))

# Prepare original points for Affine transform
pts1 = np.float32([[50,50],[200,50],[50,200]])
# Prepare future position of corresponding points after Affine transform
pts2 = np.float32([[10,100],[200,50],[100,250]])

# Create Affine rotation matrix that will take into account the previous numpy arrays created
affine_rotation_matrix = cv2.getAffineTransform(pts1,pts2)

# Transform the original image
affine_rotated = cv2.warpAffine(original, affine_rotation_matrix, (height, width))

# Show the results of the rotation
cv2.namedWindow('Doge', cv2.WINDOW_NORMAL)
cv2.imshow('Doge', original)
cv2.namedWindow('Rotated', cv2.WINDOW_NORMAL)
cv2.imshow('Rotated', rotated)
cv2.namedWindow('Affine Rotated', cv2.WINDOW_NORMAL)
cv2.imshow('Affine Rotated', affine_rotated)

# Wait until the user presses any key to continue to the next part
cv2.waitKey(0)

# Destroy all windows containing images
cv2.destroyAllWindows()
