import cv2
import time
import matplotlib.pyplot as plt

cam = cv2.VideoCapture(0)

while(True):
    result, face_image = cam.read()

    if result:
        plt.imshow(face_image)
        plt.show()

        cv2.imshow('image', face_image)
        
        cv2.waitKey(5000)
        time.sleep(5)

        face_image_gray = cv2.cvtColor(face_image, cv2.COLOR_BGR2GRAY)
        cv2.waitKey(10)

        haar_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_frontal_face_default.xml")
        cv2.waitKey(10)
        faces = haar_cascade.detectMultiScale(face_image_gray)


        for face in faces:
            center_x = face[0] + int(face[2]/2)
            center_y = face[1] + int(face[2]/2)
            if face[2] > 100:
                cv2.circle(face_image, (center_x, center_y), int(face[2] / 2), (255,0,0), 10)

        cv2.imshow('detect faces', face_image)
        time.sleep(2)
        cv2.waitKey(10)

