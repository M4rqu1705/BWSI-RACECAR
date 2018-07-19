import cv2

flags = [i for i in dir(cv2) if i.startswith('COLOR_')]

for flag in flags:
    print flag
