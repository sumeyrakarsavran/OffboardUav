#!/usr/bin/env python
# -*- coding: utf-8 -*-
# ROS python API
import rospy, cv2, time,math
import random2 as random
import numpy as np
from std_msgs.msg import Int64, Float64
from tulpar.msg import camera

liste = []
for i in range(1,1000):
    i = str(i)
    liste.append(i+'.avi')
a = random.choice(liste)

def image_publish():
    konum = camera ()
    pre_radius = 0
    image_pub = rospy.Publisher ('radius', Float64, queue_size=1)
    konum_pub = rospy.Publisher ('konum', camera, queue_size=1)
    rospy.init_node ('image_publisher', anonymous=True)
    rate = rospy.Rate (20)
    dispW = 1024
    dispH = 768
    merkezx = 512
    merkezy = 384
    flip = 2
    fourcc = cv2.VideoWriter_fourcc (*'XVID')
    out = cv2.VideoWriter (a, fourcc, 15.0, (1024, 768))
    camSet = 'nvarguscamerasrc !  video/x-raw(memory:NVMM), width=3264, height=2464, ' \
             'format=NV12, framerate=21/1 ! nvvidconv flip-method=' + str (flip) + \
             ' ! video/x-raw, width=' + str (dispW) + ', height=' + str (dispH) + \
             ', format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink'

    cap = cv2.VideoCapture (camSet)
    time.sleep (2.0)
    kernel = np.ones ((5, 5), np.float32) / 25

    if not cap.isOpened ():
        print ("error cam cannot open")
        exit ()

    while not rospy.is_shutdown ():
        ret, frame = cap.read ()
        width = int (frame.shape[1])
        height = int (frame.shape[0])
        dim = (width, height)
        frame = cv2.resize (frame, dim)
        hsv_frame = cv2.cvtColor (frame, cv2.COLOR_BGR2HSV)

        # Range for lower red
        lower_red = np.array ([0, 120, 70])
        upper_red = np.array ([10, 255, 255])
        mask1 = cv2.inRange (hsv_frame, lower_red, upper_red)

        # Range for upper range
        lower_red = np.array ([170, 120, 70])
        upper_red = np.array ([180, 255, 255])
        mask2 = cv2.inRange (hsv_frame, lower_red, upper_red)

        mask_red = mask1 + mask2
        mask_red = cv2.erode(mask_red, kernel)
        contours1 = cv2.findContours (mask_red.copy (), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]

        #cv2.rectangle (frame, (600, 480), (360, 240), (0, 255, 0), 3)

        lower_blue = np.array([94, 80, 2])
        upper_blue = np.array([120,255,255])
        mask_blue = cv2.inRange(hsv_frame,lower_blue,upper_blue)
        mask_blue =  cv2.erode(mask_blue,kernel)
        contours2 = cv2.findContours(mask_blue.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]

        """frame = cv2.line(frame, (450,330), (470,330), (0,255,0), 4)
        frame = cv2.line(frame, (450,330), (450,350), (0,255,0), 4)
        frame = cv2.line(frame, (510,330), (510,350), (0,255,0), 4)
        frame = cv2.line(frame, (510,330), (490,330), (0,255,0), 4)
        frame = cv2.line(frame, (510,390), (510,370), (0,255,0), 4)
        frame = cv2.line(frame, (510,390), (490,390), (0,255,0), 4)
        frame = cv2.line(frame, (450,390), (450,370), (0,255,0), 4)
        frame = cv2.line(frame, (450,390), (470,390), (0,255,0), 4)"""
        frame = cv2.line(frame, (512,374), (512,394), (0,255,0), 4) #center crosshair
        frame = cv2.line(frame, (502,384), (522,384), (0,255,0), 4) #center crosshair
        if len (contours1) > 0:
            # find contour which has max area
            c = max (contours1, key=cv2.contourArea)
            # find its coordinates and radius
            ((x, y), radius) = cv2.minEnclosingCircle (c)
            centerx = int (x)
            centery = int (y)
            """frame = cv2.putText(frame,'1', (500, 340), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            frame = cv2.putText(frame,'2', (440, 340), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            frame = cv2.putText(frame,'3', (440, 400), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            frame = cv2.putText(frame,'4', (500, 400), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            frame = cv2.putText(frame, 'targetx = {} '.format(centerx), (750, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            frame = cv2.putText(frame, 'targety = {} '.format(centery), (750, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)"""
            konum.bolge = int (10)
            if radius >= 0:

                frame = cv2.line(frame, ((int(x)), (int(y) + 10)), ((int(x)), (int(y) - 10)), (0, 0, 255), 5)
                frame = cv2.line(frame, ((int(x) - 10), (int(y))), ((int(x) + 10), (int(y))), (0, 0, 255), 5)
                konum.farkx = int (centerx - merkezx)
                konum.farky = int (centery - merkezy)
                r = int (math.sqrt((konum.farkx**2)+(konum.farky**2)))
                konum.bolge = int (r)
                frame = cv2.putText(frame, 'dx = {} '.format(konum.farkx), (830, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                frame = cv2.putText(frame, 'dy = {} '.format(konum.farky), (830, 125), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                frame = cv2.putText(frame, 'dist = {} '.format(r), (830, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                #out.write (frame)
                print("red detected","dx=",konum.farkx,"dy=",konum.farky,"r=",konum.bolge)
                konum_pub.publish (konum)
                rate.sleep ()
                if pre_radius < radius:
                    pre_radius = radius
                    image_pub.publish (radius)
                    rate.sleep ()
                    print ("radius=", radius)

        elif len(contours2) > 0:
            # find contour which has max area
            c = max (contours2, key=cv2.contourArea)
            # find its coordinates and radius
            ((x, y), radius) = cv2.minEnclosingCircle (c)
            centerx = int (x)
            centery = int (y)
            """frame = cv2.putText(frame,'1', (500, 340), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            frame = cv2.putText(frame,'2', (440, 340), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            frame = cv2.putText(frame,'3', (440, 400), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            frame = cv2.putText(frame,'4', (500, 400), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            frame = cv2.putText(frame, 'targetx = {} '.format(centerx), (750, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            frame = cv2.putText(frame, 'targety = {} '.format(centery), (750, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)"""
            konum.bolge = int (10)
            if radius >= 0:
                frame = cv2.line(frame, ((int(x)), (int(y) + 10)), ((int(x)), (int(y) - 10)), (0, 0, 255), 5)
                frame = cv2.line(frame, ((int(x) - 10), (int(y))), ((int(x) + 10), (int(y))), (0, 0, 255), 5)
                konum.farkx = int (centerx - merkezx)
                konum.farky = int (centery - merkezy)
                r = int (math.sqrt((konum.farkx**2)+(konum.farky**2)))
                konum.bolge = int (r)
                frame = cv2.putText(frame, 'dx = {} '.format(konum.farkx), (830, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                frame = cv2.putText(frame, 'dy = {} '.format(konum.farky), (830, 125), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                frame = cv2.putText(frame, 'dist = {} '.format(r), (830, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                #out.write (frame)
                print("blue detected","dx=",konum.farkx,"dy=",konum.farky,"r=",konum.bolge)
                konum_pub.publish (konum)
                rate.sleep ()1
    cap.release ()
    cv2.destroyAllWindows ()


if __name__ == '__main__':
    try:
        image_publish ()
    except rospy.ROSInterruptException:
        pass