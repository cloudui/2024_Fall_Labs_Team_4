import cv2
import time

image = cv2.imread("./IMG_5249.jpeg")
cv2.imshow('original', image)
time.sleep(5)
cv2.waitKey(10)

gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
cv2.imshow('grayscale', gray)
time.sleep(10)
cv2.waitKey(10)

time.sleep(10)
cv2.destroyAllWindows()