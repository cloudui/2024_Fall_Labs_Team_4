import cv2
import numpy as np
from server import detect_color

img = cv2.imread('images/green.jpg')
img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

COLORS = ["red", "red", "green", "blue"]
LOWER = [[0, 100, 100], [160, 100, 100], [35, 50, 100], [100, 100, 100]]
UPPER = [[25, 255, 255], [180, 255, 255], [85, 255, 255], [140, 255, 255]]

for i in range(len(LOWER)):
  mask = cv2.inRange(img, np.array(LOWER[i]), np.array(UPPER[i]))
  contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
  # cv2.drawContours(img, contours, -1, (255,255,255), 3)

  largest = 0
  for c in contours:
      a = cv2.contourArea(c)
      if a > largest and a > 50_000:
          largest = a

  # show masked image
  cv2.imshow('Masked Image', mask)
  cv2.waitKey(0)
  cv2.destroyAllWindows()