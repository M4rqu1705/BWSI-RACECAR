import cv2
import numpy as np

cv2.namedWindow('Video', cv2.WINDOW_NORMAL)
cv2.namedWindow('Edges', cv2.WINDOW_NORMAL)

cap = cv2.VideoCapture(0)

while True:
    # Capture the frames
    _, frame = cap.read()

    # Resize the frames
    frame = cv2.resize(frame, (0,0), fx=0.25, fy=0.25) 

    # Make variables to store image dimensions
    image_width, image_height = frame.shape[1], frame.shape[0]

    # Crop the image
    cropped_image = frame[60:120, 0:160]

    # Convert to HSV
    hsv_cropped_image = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2HSV)

    # Gaussian blur
    hsv_cropped_blurred_image = cv2.GaussianBlur(hsv_cropped_image,(5,5),0)

    lower_hsv = np.array([0, 80, 128])
    higher_hsv = np.array([74, 255, 255])

    # Color thresholding
    hsv_threshold = cv2.inRange(hsv_cropped_blurred_image, lower_hsv, higher_hsv)

    # Find the contours of the frame
    _, contours,hierarchy = cv2.findContours(hsv_threshold.copy(), 1, cv2.CHAIN_APPROX_NONE)

    # Find the biggest contour (if detected)
    if len(contours) > 0:
        biggest_contour = max(contours, key=cv2.contourArea)
        moment = cv2.moments(biggest_contour)
 
        center_x, center_y = 0, 0
        try:
            center_x = int(moment['m10']/moment['m00'])
        except ZeroDivisionError:
            center_x = 0

        try:
            center_y = int(moment['m01']/moment['m00'])
        except ZeroDivisionError:
            center_y = 0
 
        cv2.line(cropped_image,(center_x,0),(center_x,image_height),(255,0,0),1)
        cv2.line(cropped_image,(0,center_y),(image_width,center_y),(255,0,0),1)
 
        cv2.drawContours(cropped_image, contours, -1, (0,0,255), 1)

        print "Error = %s" % (str(80-center_x))
 

    #Display the resulting frame

    cv2.imshow('Video', hsv_threshold)
    cv2.imshow('Edges', cropped_image)
    cv2.waitKey(1)

cv2.destroyAllWindows
