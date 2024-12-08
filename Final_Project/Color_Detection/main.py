import cv2
import time
import matplotlib.pyplot as plt
import numpy as np

cam = cv2.VideoCapture(0)
i = 0 #too dark on first capture

# hsv thresholds in this order: red, blue, green, other
lower = [[0, 19, 0], [107, 130, 62], [41, 29, 78], []]
upper = [[28, 255, 255], [121, 255, 255], [96, 214, 255], []]

colors = ["red", "blue", "green", "other"]

while(True):
    result, image = cam.read()
    color_areas = []

    if result and i != 0:
        image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        for i in range(3): #change to 4
            mask = cv2.inRange(image_hsv, np.array(lower[i]), np.array(upper[i]))
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            largest = 0
            for c in contours:
                a = cv2.contourArea(c)
                if a > largest:
                    largest = a

            color_areas.append(largest)

        max_index = 0
        for i in range(1, 3):
            if color_areas[i] > color_areas[max_index]:
                max_index = i

        print("Color detected: " + colors[max_index])

        time.sleep(5)
        cv2.destroyAllWindows()
    
    i = 1
