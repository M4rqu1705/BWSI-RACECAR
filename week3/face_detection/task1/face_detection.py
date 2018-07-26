import cv2
import numpy as np

# Read the image to analyze
image = cv2.imread('cruz.png')

# Convert image to grayscale to make process more efficient
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# Initialize variable called recognized_face to later contain Carlos's face
recognized_face = None

# Load the required XML classifiers
haar = cv2.CascadeClassifier('../practices/haarcascade_frontalface_default.xml')

# Detect faces
faces = haar.detectMultiScale(gray, 1.1, 5)

# Check if a face was found
if len(faces)>0:
    # Iterate through the faces list
    for face in faces:
        # Recuperate face's x, y position and height and width
        x, y, width, height = face
        # Store the face into the recognized_face variable
        recognized_face = image[y:y+height, x:x+width]

# Show the whole image and the face
cv2.imshow('Image', image)
cv2.imshow('Face', recognized_face)

# Store the face in a png file
cv2.imwrite('cruz_face.png', recognized_face)

cv2.waitKey(0)
cv2.destroyAllWindows()
