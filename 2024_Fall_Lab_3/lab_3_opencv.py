import cv2
import time

image = cv2.imread("./IMG_5249.jpeg")
cv2.imshow('original', image)
time.sleep(5)
cv2.waitKey(10)

gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
cv2.imshow('grayscale', gray)
time.sleep(5)
cv2.waitKey(10)

edge = cv2.Canny(image, 100, 150)
cv2.waitKey(10)
cv2.imshow('edge detection', edge)
time.sleep(5)
cv2.waitKey(10)

face_image = cv2.imread("./IMG_5250.jpeg")
face_image_gray = cv2.cvtColor(face_image, cv2.COLOR_BGR2GRAY)
cv2.waitKey(10)

haar_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
cv2.waitKey(10)
faces = haar_cascade.detectMultiScale(face_image_gray)

for face in faces:
    center_x = face[0] + int(face[2]/2)
    center_y = face[1] + int(face[2]/2)
    if face[2] > 100:
        cv2.circle(face_image, (center_x, center_y), int(face[2]/2), (255,0,0), 10)

cv2.imshow('detect faces', face_image)
time.sleep(5)
cv2.waitKey(10)

time.sleep(10)
cv2.destroyAllWindows()