#!/usr/bin/env python
# ROS python API
#coding-*-coding: utf-8 -*-
import rospy, cv2,time
import numpy as np
from std_msgs.msg import String

lower_red = (161,155,84)
upper_red = (179, 255, 255)
fourcc=cv2.VideoWriter_fourcc(*'XVID')
dispW=960
dispH=720
flip=2
camSet='nvarguscamerasrc !  video/x-raw(memory:NVMM), width=3264, height=2464, ' \
       'format=NV12, framerate=21/1 ! nvvidconv flip-method='+str(flip)+\
       ' ! video/x-raw, width='+str(dispW)+', height='+str(dispH)+\
       ', format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink'

out = cv2.VideoWriter('output.avi', fourcc, 30.0, (960, 720))

def image_publish():
	pre_radius = 0
	# open the camera
	cam = cv2.VideoCapture (camSet)
	time.sleep (2.0)

	# check if camera opened without error
	if not cam.isOpened ():
		print ("cam is not opened")
		exit ()

	while True:
		image_pub = rospy.Publisher ('radius', String, queue_size=10)
		konum_pub = rospy.Publisher ('konum', String, queue_size=10)
		rospy.init_node ('image_publisher', anonymous=True)
		rate = rospy.Rate (20)
		# capture the frame
		ret, frame = cam.read ()
		# width = int(frame.shape[1] * scale_percent / 100)
		# height = int(frame.shape[0] * scale_percent / 100)
		width = int (frame.shape[1])
		height = int (frame.shape[0])

		# resize frame
		dim = (width, height)
		frame = cv2.resize (frame, dim)

		gray_frame = cv2.cvtColor (frame, cv2.COLOR_BGR2GRAY)
		blurred_org_frame = cv2.GaussianBlur (frame, (11, 11), 0)
		hsv = cv2.cvtColor (blurred_org_frame, cv2.COLOR_BGR2HSV)
		mask = cv2.inRange (hsv, lower_red, upper_red)
		mask = cv2.erode (mask, None, iterations=2)
		mask = cv2.dilate (mask, None, iterations=2)
		net = cv2.bitwise_and (frame, frame, mask=mask)
		# frame = cv2.medianBlur(frame, 15)
		gray_frame = cv2.medianBlur (gray_frame, 5)
		rows = gray_frame.shape[0]
		# cv2.circle(frame, (192,144), 30, (0, 255, 0), 3)
		cv2.rectangle (frame, (600, 480), (360, 240), (0, 255, 0), 3)
		circles = cv2.HoughCircles (gray_frame, cv2.HOUGH_GRADIENT, 1, rows / 8, param1=100, param2=30, minRadius=10,
									maxRadius=2000)
		out.write (frame)
		if circles is not None:
			circles = np.uint16 (np.around (circles))
			for i in circles[0, :]:
				center = (i[0], i[1])
				# circle outline
				radius = i[2]
				cv2.circle (frame, center, radius, (255, 0, 255), 3)

				centerx = center[0]
				centery = center[1]
				if pre_radius < radius:
					pre_radius = radius
					radius_data = str (radius)
					image_pub.publish (radius_data)
					print ("radius=", radius)
					rate.sleep ()
				for j in center:

					if (centerx < 430 and centery < 310):
						konum=1
						print (konum)
						konum_pub.publish(konum)
					elif (centerx > 430 and centery < 310 and centerx < 530):
						konum=2
						print (konum)
						konum_pub.publish(konum)
					elif (centerx > 530 and centery < 310):
						konum=3
						print (konum)
						konum_pub.publish(konum)

					elif (centerx > 530 and centery > 310 and centery < 410):
						konum=4
						print (konum)
						konum_pub.publish(konum)
					elif (centerx > 530 and centery > 410):
						konum=5
						print (konum)
						konum_pub.publish(konum)

					elif (centerx < 530 and centery > 410 and centerx > 430):
						konum=6
						print (konum)
						konum_pub.publish(konum)


					elif (centerx < 430 and centery > 410):
						konum=7
						print (konum)
						konum_pub.publish(konum)

					elif (centerx < 430 and centery < 410 and centery > 310):
						konum=8
						print (konum)
						konum_pub.publish(konum)

					elif centerx > 430 and centerx < 530 and centery < 410 and centery > 310:
						print ("merkez")
						konum=0
						konum_pub.publish(konum)

	cam.release ()
	cv2.destroyAllWindows ()


if __name__ == '__main__':
    try:
        image_publish()
    except rospy.ROSInterruptException:
        pass
