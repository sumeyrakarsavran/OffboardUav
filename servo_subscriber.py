#!/usr/bin/env python
# -*- coding: utf-8 -*-
# ROS python API
import rospy, time
import numpy as np
from std_msgs.msg import Int64, Float64
import Jetson.GPIO as GPIO
outputPin = 33
servoDurum=0

GPIO.setmode(GPIO.BOARD)
GPIO.setup(outputPin, GPIO.OUT, initial=GPIO.HIGH)
p2 = GPIO.PWM(outputPin, 50)

def servoCallback(data):
    global servoDurum
    servoDurum= int(data.data)
    print(servoDurum)

def main():
    global servoDurum
    while True:
        if servoDurum==0:

            rate = rospy.Rate (5.0)
            rospy.Subscriber ('servo', Int64, servoCallback)
            rate.sleep ()

        elif servo_durum==1:
            p2.start(12)
            time.sleep(0.75)
            p2.start(2)
            time.sleep(0.75)
            p2.start(12)
            time.sleep(0.75)
            p2.start(2)
            time.sleep(0.75)
            p2.stop()
            GPIO.cleanup()
            break


if __name__ == '__main__':

    rospy.init_node ('servo_durum1', anonymous=True)
    rospy.Subscriber ('servo', Int64, servo_callback)

    main()
