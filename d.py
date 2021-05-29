#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
#kamera genişliği
dispW=960 
#kamera yüksekliği
dispH=720 
flip=4 
#flipe 4 atanır kamera düz görüntü verir
camSet='nvarguscamerasrc !  video/x-raw(memory:NVMM), width=3264, height=2464, ' \
       'format=NV12, framerate=21/1 ! nvvidconv flip-method='+str(flip)+\
       ' ! video/x-raw, width='+str(dispW)+', height='+str(dispH)+\
       ', format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink'

cap = cv2.VideoCapture(camSet)

kernel = np.ones((5,5),np.float32)/25

if not cap.isOpened():
    print("error cam cannot open")
    exit()
while True:
    ret, frame = cap.read()
#    frame = cv2.GaussianBlur(frame, (5, 5), 0)
    #frame = cv2.bilateralFilter(frame, 9, 75, 75) its slow compared to others
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    lower_blue = np.array([161,155,84])
    upper_blue = np.array([179, 255, 255])
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    res = cv2.bitwise_and(frame, frame, mask=mask)

    cv2.imshow('res', mask)
    print("blue detected")
    if cv2.waitKey(1) == ord('q'):
        break


cap.release()
cv2.destroyAllWindows()
