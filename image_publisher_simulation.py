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
    rate = rospy.Rate (5)
    dispW = 960
    dispH = 720
    merkezx = 480
    merkezy = 360
    flip = 2
    fourcc = cv2.VideoWriter_fourcc (*'XVID')
    out = cv2.VideoWriter (a, fourcc, 15.0, (960, 720))
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

        mask = mask1 + mask2
        #cv2.rectangle (frame, (600, 480), (360, 240), (0, 255, 0), 3)
        contours = cv2.findContours (mask.copy (), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
        frame = cv2.line(frame, (430,310), (460,310), (0,255,0), 4)
        frame = cv2.line(frame, (430,310), (430,340), (0,255,0), 4)
        frame = cv2.line(frame, (530,310), (500,310), (0,255,0), 4)
        frame = cv2.line(frame, (530,310), (530,340), (0,255,0), 4)
        frame = cv2.line(frame, (530,410), (530,380), (0,255,0), 4)
        frame = cv2.line(frame, (530,410), (500,410), (0,255,0), 4)
        frame = cv2.line(frame, (430,410), (460,410), (0,255,0), 4)
        frame = cv2.line(frame, (430,410), (430,380), (0,255,0), 4) 
        frame = cv2.line(frame, (480,350), (480,370), (0,255,0), 4) #center crosshair
        frame = cv2.line(frame, (470,360), (490,360), (0,255,0), 4) #center crosshair
        if len (contours) > 0:
            # find contour which has max area
            c = max (contours, key=cv2.contourArea)
            # find its coordinates and radius
            ((x, y), radius) = cv2.minEnclosingCircle (c)
            centerx = int (x)
            centery = int (y)
            frame = cv2.putText(frame,'1', (500, 340), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            frame = cv2.putText(frame,'2', (440, 340), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            frame = cv2.putText(frame,'3', (440, 400), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            frame = cv2.putText(frame,'4', (500, 400), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            frame = cv2.putText(frame, 'targetx = {} '.format(centerx), (750, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            frame = cv2.putText(frame, 'targety = {} '.format(centery), (750, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            konum.bolge = int (10)
            if radius >= 0:

                frame = cv2.line(frame, ((int(x)), (int(y) + 10)), ((int(x)), (int(y) - 10)), (0, 0, 255), 5)
                frame = cv2.line(frame, ((int(x) - 10), (int(y))), ((int(x) + 10), (int(y))), (0, 0, 255), 5)
		konum.farkx = int (centerx - merkezx)
                konum.farky = int (centery - merkezy)
       	    	r = math.sqrt((konum.farkx**2)+(konum.farky**2))
                konum.bolge = int (r)
            	frame = cv2.putText(frame, 'dx = {} '.format(konum.farkx), (750, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            	frame = cv2.putText(frame, 'dy = {} '.format(konum.farky), (750, 125), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            	frame = cv2.putText(frame, 'distance = {} '.format(r), (750, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            	out.write (frame)
		print(konum.farkx,konum.farky,konum.bolge)
                konum_pub.publish (konum)
                rate.sleep ()
                if pre_radius < radius:
		    rate = rospy.Rate (30)
                    pre_radius = radius
                    image_pub.publish (radius)
                    rate.sleep ()
                    print ("radius=", radius)


                """if (480 < centerx and centery < 340 or centery < 360 and 500 < centerx):
                    konum.bolge = int (1)
                    konum.farkx = int (centerx - merkezx)
                    konum.farky = int (merkezy - centery)
                    print (konum.bolge, konum.farkx, konum.farky)
                    konum_pub.publish (konum)
                    rate.sleep ()


                elif (centerx < 460 and centery < 360 or centery < 340 and centerx < 480):
                    konum.bolge = int (2)
                    konum.farkx = int (merkezx - centerx)
                    konum.farky = int (merkezy - centery)
                    print (konum.bolge, konum.farkx, konum.farky)
                    konum_pub.publish (konum)
                    rate.sleep ()

                elif (centerx < 460 and 360 < centery or 380 < centery and centerx < 480):
                    konum.farkx = int (merkezx - centerx)
                    konum.farky = int (centery - merkezy)
                    konum.bolge = int (3)
                    print (konum.bolge, konum.farkx, konum.farky)
                    konum_pub.publish (konum)
                    rate.sleep ()

                elif (480 < centerx and 380 < centery or 360 < centery and 490 < centerx):
                    konum.farkx = int (centerx - merkezx)
                    konum.farky = int (centery - merkezy)
                    konum.bolge = int (4)
                    print (konum.bolge, konum.farkx, konum.farky)
                    konum_pub.publish (konum)
                    rate.sleep ()

                elif (460 <= centerx <= 500 and 320 <= centery <= 380):
                    konum.farkx = int (centerx - merkezx)
                    konum.farky = int (centery - merkezy)
                    konum.bolge = int (0)
                    print (konum.bolge, konum.farkx, konum.farky)
                    konum_pub.publish (konum)
                    rate.sleep ()"""
    cap.release ()
    cv2.destroyAllWindows ()


if __name__ == '__main__':
    try:
        image_publish ()
    except rospy.ROSInterruptException:
        pass
