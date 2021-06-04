#!/usr/bin/env python
# ROS python API
#coding-*-coding: utf-8 -*-
import rospy, cv2
import numpy as np
from std_msgs.msg import String


def image_publish():
	pre_radius = 0
	image_pub=rospy.Publisher('chatter',String,queue_size=10)
	rospy.init_node('image_publisher',anonymous=True)
	rate=rospy.Rate(20)
	dispW=960 
	dispH=720 
	flip=4 
	camSet='nvarguscamerasrc !  video/x-raw(memory:NVMM), width=3264, height=2464, ' \
	       'format=NV12, framerate=21/1 ! nvvidconv flip-method='+str(flip)+\
	       ' ! video/x-raw, width='+str(dispW)+', height='+str(dispH)+\
	       ', format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink'

	cap = cv2.VideoCapture(camSet)
	kernel = np.ones((5,5),np.float32)/25
	if not cap.isOpened():
	    print("error cam cannot open")
	    exit()
	while not rospy.is_shutdown():
	        lower_red = (0, 80, 80)
        	upper_red = (0, 255, 255)
		scale_percent = 50
	    	ret, frame = cap.read()
#resize frame
        	width = int(frame.shape[1] * scale_percent / 100)
        	height = int(frame.shape[0] * scale_percent / 100)
      		dim = (width, height)
       		frame = cv2.resize(frame, dim)
        	blurred_org_frame = cv2.GaussianBlur(frame, (11, 11), 0)
        	hsv_frame = cv2.cvtColor(blurred_org_frame, cv2.COLOR_BGR2HSV)
        	mask = cv2.inRange(hsv_frame, lower_red, upper_red)
        	mask = cv2.erode(mask, None, iterations=2)
        	mask = cv2.dilate(mask, None, iterations=2)
        	contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]

        	if len(contours) > 0:

	            # find contour which has max area
         	   c = max(contours, key=cv2.contourArea)
         	   # find its coordinates and radius
         	   ((x, y), radius) = cv2.minEnclosingCircle(c)
         	   if radius > 10:
                # draw circle around blue color
                #cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                #cv2.circle(mask, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                # draw contours around blue color
               	#cv2.drawContours(frame, contours, -1, (0, 255, 255), 2)
			if pre_radius < radius:
        			pre_radius=radius
		       		radius_data=str(radius)	
		        	image_pub.publish(radius_data)
         	        	print("radius=", radius)
		        	rate.sleep()
	cap.release()
    	cv2.destroyAllWindows()























if __name__ == '__main__':
    try:
        image_publish()
    except rospy.ROSInterruptException:
        pass
