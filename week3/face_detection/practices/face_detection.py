import cv2              # Import cv2 module for image analysis
import numpy as np      # Import numpy functions to complement cv2

# Instantiate VideoCapture object with the laptop webcam
cap = cv2.VideoCapture(0)

# Make named windows of normal size for future use
cv2.namedWindow('Gaussian Blur', cv2.WINDOW_NORMAL)
cv2.namedWindow('Median Blur', cv2.WINDOW_NORMAL)

# Make sure openCV is running optimized code. Increases efficiency
cv2.setUseOptimized(True)

# Load the required XML classifier
face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')

while True:
    # Read from webcam
    _, frame = cap.read()
    # Resize image for faster processing
    frame = cv2.resize(frame, (0,0), fx=0.25, fy=0.25)
    
    # Use two types of blur with the original frame: median blur and Gaussian blur
    median, gaussian = cv2.medianBlur(frame, 5), cv2.GaussianBlur(frame, (5, 5), 0)

    # Convert image of interest to grayscale for faster processing
    gray = cv2.cvtColor(median, cv2.COLOR_BGR2GRAY)

    # Detect faces in different parts of the image
    faces = face_cascade.detectMultiScale(gray, 1.1, 5)

    # Check if a face was found. Otherwise do not display it. It would provoke an error
    if len(faces) > 0:
        # Run through the x, y, width and height of every detected face
        for (x,y,w,h) in faces:
            # Make a white rectangle around detected faces
            cv2.rectangle(median, (x, y), (x+w, y+h), (255, 255, 255), 2)

    
    # Show images
    cv2.imshow('Gaussian Blur', gaussian)
    cv2.imshow('Median Blur', median)

    cv2.waitKey(1)

cv2.destroyAllWindows()
