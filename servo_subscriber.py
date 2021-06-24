#!/usr/bin/env python
# -*- coding: utf-8 -*-
# ROS python API
import rospy, time
import numpy as np
from std_msgs.msg import Int64, Float64
import Jetson.GPIO as GPIO
output_pin2 = 33


GPIO.setmode(GPIO.BOARD)
GPIO.setup(output_pin2, GPIO.OUT, initial=GPIO.HIGH)
p2 = GPIO.PWM(output_pin2, 50)
servo_durum=0

def servo_callback(data):
    global servo_durum
    servo_durum= int(data.data)
    print(servo_durum)
def main():
    global servo_durum
    while True:
        if servo_durum==0:
            rospy.Subscriber ('servo', Int64, servo_callback)

        elif servo_durum==1:
            p2.start (15)
            time.sleep (0.5)
            p2.stop ()
            GPIO.cleanup ()
            break
if __name__ == '__main__':
    rospy.init_node ('servo_durum1', anonymous=True)

    rospy.Subscriber ('servo', Int64, servo_callback)
    main()