#!/usr/bin/env python

# ROS python API
#coding-*-coding: utf-8 -*-
import rospy, cv2,time
import numpy as np
from std_msgs.msg import String

def image_publish():
	pre_radius = 0
	image_pub = rospy.Publisher ('radius', String, queue_size=10)
	konum_pub = rospy.Publisher ('konum', String, queue_size=10)
	rospy.init_node ('image_publisher', anonymous=True)
	rate=rospy.Rate(20)
	dispW=960
	dispH=720
	flip=2
	#fourcc = cv2.VideoWriter_fourcc (*'XVID')
	#out = cv2.VideoWriter ('output.avi', fourcc, 24.0, (960, 720))
	camSet='nvarguscamerasrc !  video/x-raw(memory:NVMM), width=3264, height=2464, ' \
	       'format=NV12, framerate=21/1 ! nvvidconv flip-method='+str(flip)+\
	       ' ! video/x-raw, width='+str(dispW)+', height='+str(dispH)+\
	       ', format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink'

	cap = cv2.VideoCapture(camSet)
	time.sleep (2.0)
	kernel = np.ones((5,5),np.float32)/25
	if not cap.isOpened():
		print("error cam cannot open")
		exit()
	while not rospy.is_shutdown():
			lower_red = (161,155,84)
			upper_red = (179, 255, 255)
			ret, frame = cap.read()
			width = int (frame.shape[1])
			height = int (frame.shape[0])
			dim = (width, height)
			frame = cv2.resize(frame, dim)
			blurred_org_frame = cv2.GaussianBlur(frame, (11, 11), 0)
			hsv_frame = cv2.cvtColor(blurred_org_frame, cv2.COLOR_BGR2HSV)
			mask = cv2.inRange(hsv_frame, lower_red, upper_red)
			mask = cv2.erode(mask, None, iterations=2)
			mask = cv2.dilate(mask, None, iterations=2)
			cv2.rectangle (frame, (600, 480), (360, 240), (0, 255, 0), 3)
			contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
	#		out.write (frame)

			if len(contours) > 0:
				print("contours sıfırdan buyuk")
				# find contour which has max area
				c = max(contours, key=cv2.contourArea)
				# find its coordinates and radius
				((x, y), radius) = cv2.minEnclosingCircle(c)
				centerx=x
				centery=y
				if radius > 10:

					if pre_radius < radius:
						pre_radius=radius
						radius_data=str(radius)
						image_pub.publish(radius_data)
						print("radius=", radius)
						rate.sleep()

					if (centerx < 360 and centery < 240):
						konum = 1
						print (konum)
						konum_data = str (konum)
						konum_pub.publish (konum_data)
					elif(centerx > 360 and centery < 240 and centerx < 600):
						konum = 2
						print (konum)
						konum_data = str (konum)
						konum_pub.publish (konum_data)
					elif (centerx > 600 and centery < 240):
						konum = 3
						print (konum)
						konum_data = str (konum)
						konum_pub.publish (konum_data)
					elif (centerx > 600 and centery > 240 and centery < 480):
						konum = 4
						print (konum)
						konum_data = str (konum)
						konum_pub.publish (konum_data)
					elif (centerx > 600 and centery > 480):
						konum = 5
						print (konum)
						konum_data = str (konum)
						konum_pub.publish (konum_data)
					elif (centerx < 600 and centery > 480 and centerx > 360):
						konum = 6
						print (konum)
						konum_data = str (konum)
						konum_pub.publish (konum_data)
					elif (centerx < 360 and centery > 480):
						konum = 7
						print (konum)
						konum_data = str (konum)
						konum_pub.publish (konum_data)
					elif (centerx < 360 and centery < 480 and centery > 240):
						konum = 8
						print (konum)
						konum_data = str (konum)
						konum_pub.publish (konum_data)
					elif centerx > 360 and centerx < 600 and centery < 480 and centery > 240:
						print ("merkez")
						konum = 0
						konum_data = str (konum)
						konum_pub.publish (konum_data)
	cap.release()
	cv2.destroyAllWindows()




if __name__ == '__main__':
	try:
		image_publish()
	except rospy.ROSInterruptException:
		pass
