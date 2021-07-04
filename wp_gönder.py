#!/usr/bin/env python
# -*- coding: utf-8 -*-
# ROS python API
import rospy, mavros, math, time
# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped, Twist, TwistStamped
from sensor_msgs.msg import NavSatFix, Image
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from std_msgs.msg import String, Float64, Int64
from decimal import *
from cv_bridge import CvBridge, CvBridgeError
from tulpar.msg import camera
import Jetson.GPIO as GPIO

# Message publisher for local velocity
velocity_pub = rospy.Publisher ('mavros/setpoint_raw/local', PositionTarget, queue_size=10)
msg1 = PositionTarget ()
z_pub = rospy.Publisher ('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=10)
msg2 = Twist ()
# Current Position
latitude = 0
longitude = 0
altitude = 0
altitude1 = 0
local_x = 0
local_y = 0
# Position before Move function execute
previous_latitude = 0
previous_longitude = 0
previous_altitude = 0.0
global sp_pub, sp_glob_pub


def altitude_callback(data):
    global altitude1
    altitude1 = data.relative


mavros.set_namespace ()  # for global callback


def globalPositionCallback(globalPositionCallback):
    global latitude, longitude, altitude
    latitude = globalPositionCallback.latitude
    longitude = globalPositionCallback.longitude
    altitude = globalPositionCallback.altitude


amsl = 0


def amslcallback(data):
    global amsl
    amsl = float ("{0:.1f}".format (data.amsl))


def localPositionCallback(localPositionCallback):
    global local_x, local_y, local_w
    local_x = localPositionCallback.pose.position.x
    local_y = localPositionCallback.pose.position.y


def glob_pos_pub(wp_lat, wp_long, wp_alt):
    cnt = Controller ()
    global altitude, latitude, longitude, sp_glob_pub, amsl
    cnt.sp_glob.latitude = wp_lat
    cnt.sp_glob.longitude = wp_long
    cnt.sp_glob.altitude = amsl + wp_alt
    rate = rospy.Rate (20.0)

    while not rospy.is_shutdown ():
        rate.sleep ()
        sp_glob_pub.publish (cnt.sp_glob)
        latitude = float ("{0:.6f}".format (latitude))
        longitude = float ("{0:.6f}".format (longitude))
        # print (latitude, wp_lat, longitude, wp_long, cnt.sp_glob.altitude, amsl)
        # print("poselanıyor")
        # print(longitude,latitude,altitude,cnt.sp_glob.altitude)
        if (latitude - 0.000001) < wp_lat < (latitude + 0.000001) and (longitude - 0.000002) < wp_long < (
                longitude + 0.000002) and (amsl - 0.5) < cnt.sp_glob.altitude < (amsl + 0.5):
            print ("Konuma gidildi.")
            break


# cv2 bridge
bridge = CvBridge ()
cv_image = ""
last_red_latitude = 0
last_red_longitude = 0
red_latitude = 0
red_longitude = 0
red_latitude2 = 0
red_longitude2 = 0
pre_radius = 0


def image_callback(radius):
    global red_latitude, red_longitude, latitude, longitude, pre_radius
    pre_radius = radius
    red_latitude = float ("{0:.6f}".format (latitude))
    red_longitude = float ("{0:.6f}".format (longitude))
    print ("*************GOT DATA*************", radius, red_latitude, red_longitude)
    rate = rospy.Rate (20)
    rate.sleep ()


konum = 100
farkx = 0
farky = 0


def cam_konum_callback(data):
    global konum, farkx, farky
    konum = int (data.bolge)
    farkx = int (data.farkx)
    farky = int (data.farky)
    rate = rospy.Rate (20)
    rate.sleep ()
    print (konum, farkx, farky )


# Flight modes class
class fcuModes:
    def __init__(self):
        pass

    def setTakeoff(self):
        global longitude, latitude
        rospy.wait_for_service ('mavros/cmd/takeoff')
        try:
            takeoffService = rospy.ServiceProxy (
                '/mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
            takeoffService (altitude=0, latitude=latitude,
                            longitude=longitude, min_pitch=0, yaw=0)
        except rospy.ServiceException as e:
            print ("Service takeoff call failed: %s" % e)

    def setArm(self):
        rospy.wait_for_service ('mavros/cmd/arming')
        try:
            print ("Waiting for arming...")
            armService = rospy.ServiceProxy ('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService (True)
        except rospy.ServiceException, e:
            print ("Service arming call failed: %s" % e)

    def setDisarm(self):
        rospy.wait_for_service ('mavros/cmd/arming')
        try:
            print ("Waiting for disarming...")
            armService = rospy.ServiceProxy ('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService (False)
        except rospy.ServiceException, e:
            print ("Service disarming call failed: %s" % e)

    def setStabilizedMode(self):
        rospy.wait_for_service ('mavros/set_mode')
        try:
            print ("It's stabilazed mode!")
            flightModeService = rospy.ServiceProxy ('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService (custom_mode='STABILIZED')
        except rospy.ServiceException, e:
            print ("service set_mode call failed: %s. Stabilized Mode could not be set." % e)

    def setOffboardMode(self):
        global sp_glob_pub
        rospy.wait_for_service ('/mavros/set_mode')
        cnt = Controller ()
        rate = rospy.Rate (5.0)
        k = 0
        while k < 12:
            sp_glob_pub.publish (cnt.sp_glob)
            rate.sleep ()
            k = k + 1
            rospy.wait_for_service ('/mavros/set_mode')

        try:
            flightModeService = rospy.ServiceProxy ('/mavros/set_mode', mavros_msgs.srv.SetMode)
            response = flightModeService (custom_mode='OFFBOARD')
            return response.mode_sent
        except rospy.ServiceException as e:
            print ("service set_mode call failed: %s. Offboard Mode could not be set." % e)
            return False

    def setAltitudeMode(self):
        rospy.wait_for_service ('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy ('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService (custom_mode='ALTCTL')
        except rospy.ServiceException, e:
            print ("service set_mode call failed: %s. Altitude Mode could not be set." % e)

    def setLoiterMode(self):
        rospy.wait_for_service ('/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy ('/mavros/set_mode', mavros_msgs.srv.SetMode)
            isModeChanged = flightModeService (custom_mode='AUTO.LOITER')  # return true or false
        except rospy.ServiceException as e:
            print (
                "service set_mode call failed: %s. AUTO.LOITER Mode could not be set. Check that GPS is enabled %s" % e)

    def setPositionMode(self):
        rospy.wait_for_service ('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy ('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService (custom_mode='POSCTL')
        except rospy.ServiceException, e:
            print ("service set_mode call failed: %s. Position Mode could not be set." % e)

    def setLandMode(self):
        global pos_mode
        pos_mode = False
        rospy.wait_for_service ('/mavros/cmd/land')
        try:
            landService = rospy.ServiceProxy ('/mavros/cmd/land', mavros_msgs.srv.CommandTOL)
            isLanding = landService (altitude=0)
        except rospy.ServiceException, e:
            print
            "service land call failed: %s. The vehicle cannot land " % e


class Controller:

    # initialization method
    def __init__(self):
        global local_x, local_y, local_w
        # Drone state
        self.state = State ()  # using that msg for send few setpoint messages, then activate OFFBOARD mode, to take effect
        # Instantiate a setpoints message
        self.sp_glob = GlobalPositionTarget ()
        self.sp_glob.type_mask = int ('010111111000', 2)
        self.sp_glob.coordinate_frame = 6  # FRAME_GLOBAL_INT
        self.sp_glob.latitude = 0
        self.sp_glob.longitude = 0
        self.sp_glob.altitude = 0
        self.level = PoseStamped ()
        self.sp = PositionTarget ()

        self.local_position_publisher = rospy.Publisher (mavros.get_topic ('setpoint_position', 'local'), PoseStamped,
                                                         queue_size=10)
        # set the flag to use position setpoints and yaw angle
        # self.wp.position.z = self.ALT_SP
        self.local_pos = Point (0, 0, 0)
        self.sp_pub = rospy.Publisher ('mavros/setpoint_raw/local', PositionTarget, queue_size=1)
        self.rate = rospy.Rate (5.0)

    ## Drone State callback
    def stateCb(self, msg):
        self.state = msg

    def updateSp(self):
        self.sp.position.x = local_x
        self.sp.position.y = local_y


def alcal(alt):
    global altitude1
    print ("ALCALIYOR")
    rate = rospy.Rate (10.0)
    ALT_SP = alt
    msg2.linear.z = -1.5
    while not rospy.is_shutdown ():
        print ("Suanki Yukseklik", altitude1)
        z_pub.publish (msg2)
        rate.sleep ()
        if (ALT_SP + 0.15) > altitude1:
            print ("ALCALDIGI DEGER=", altitude1)
            break
    msg2.linear.z = 0.
    for i in range (100):
        z_pub.publish (msg2)
    rate.sleep ()


def yuksel():
    global altitude1
    print ("YUKSELIYOR")
    rate = rospy.Rate (10.0)
    ALT_SP = 6
    msg2.linear.z = 3
    while not rospy.is_shutdown ():
        print ("Suanki Yukseklik", altitude1)
        if (ALT_SP - 0.15) < altitude1:
            print ("YUKSELDIGI DEGER=", altitude1)
            break

        z_pub.publish (msg2)
        rate.sleep ()

    msg2.linear.z = 0.
    for i in range (100):
        z_pub.publish (msg2)
    rate.sleep ()


def movingcenter():
    global konum, msg1, velocity_pub, farkx, farky, red_longitude2, red_latitude2, longitude, latitude
    modes = fcuModes ()
    rate = rospy.Rate (5)
    while 1:
        #v = (0.1 + (0.000833 * konum)) #Vmax =0.6
        v = (0.05 + (0.0009167 * konum)) #Vmax=0.6
        dist=19
        count=0
        if konum >= 25:
            msg1.velocity.z = 0
            msg1.header.stamp = rospy.get_rostime ()
            msg1.header.frame_id = "local_ned"
            msg1.coordinate_frame = 8
            msg1.type_mask = int ('011111000111', 2)

            if farkx <= -dist:
                msg1.yaw = 0.0  # rad
                msg1.yaw_rate = 0.09  # (rad/sn)
                velocity_pub.publish (msg1)
                rate.sleep ()
            elif farkx >= dist:
                msg1.yaw = 0  # rad
                msg1.yaw_rate = -0.09   # (rad/sn)
                velocity_pub.publish (msg1)
                rate.sleep ()
            elif -dist < farkx < dist:
                msg1.yaw = 0  # rad
                msg1.yaw_rate = 0
                velocity_pub.publish (msg1)
                rate.sleep ()

            """if farkx <= -dist :
                msg1.velocity.y = -v
                velocity_pub.publish (msg1)
                rate.sleep ()

            elif farkx >= dist:
                msg1.velocity.y = v
                velocity_pub.publish (msg1)
                rate.sleep ()

            elif -dist < farkx < dist:
                msg1.velocity.y = 0
                velocity_pub.publish (msg1)
                rate.sleep ()"""

            if farky <= -dist:
                msg1.velocity.x = -v
                velocity_pub.publish (msg1)
                rate.sleep ()

            elif farky >= dist:
                msg1.velocity.x = v
                velocity_pub.publish (msg1)
                rate.sleep ()

            elif -dist < farky < dist:
                msg1.velocity.x = 0
                velocity_pub.publish (msg1)
                rate.sleep ()

        elif konum < 25:
            msg1.velocity.z = 0
            msg1.velocity.y = 0
            msg1.velocity.x = 0
            msg1.yaw = 0  # rad
            msg1.yaw_rate = 0
            velocity_pub.publish (msg1)
            rate.sleep ()
            break


def waypointmove():
    output_pin = 18
    rate = rospy.Rate (20.0)
    global red_longitude, red_latitude
    modes = fcuModes ()
    """glob_pos_pub (41.090384, 28.617784 , 0) #blue lat long
    glob_pos_pub (41.090520, 28.617332 , 0) #2. direk lat long
    glob_pos_pub (41.090384, 28.617784 , 0) #blue lat long
    movingcenter ()  # maviyi ortala alçal yüksel
    alcal (1.6)
    print ("*******ALCALDI*******")
    modes.setLoiterMode ()
    # PUMP
    GPIO.setmode (GPIO.BCM)
    GPIO.setup (output_pin, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.output (output_pin, GPIO.HIGH) #SUYU AL
    rospy.sleep (17)
    GPIO.output (output_pin, GPIO.LOW) #SUYU ALMAYI DURDUR
    GPIO.cleanup ()
    print("SU ALINDI")
    modes.setOffboardMode ()
    yuksel ()
    print ("*******YÜKSELDİ*******")
    glob_pos_pub( red_latitude,red_longitude,0) # red lat long"""
    movingcenter ()  # kırmızıyı ortala alçal yüksel
    alcal (4)
    print ("4 metreye alçaldı")
    modes.setLoiterMode ()
    """s = int (1)
    servo_pub = rospy.Publisher ('servo', Int64, queue_size=1)
    for i in range(10):
        servo_pub.publish (s)
        rate.sleep()
    rospy.sleep (5)
    print ("SU BIRAKILIYOR")
    modes.setOffboardMode ()
    yuksel ()
    glob_pos_pub( 41.0903366 ,28.617699,0) #FARKLI BİR YERE LAND İÇİN GİT"""
    rospy.sleep (5)
    modes.setLandMode ()


# Main function
def main():
    global sp_pub, sp_glob_pub, amsl
    # initiate node
    rospy.init_node ('setpoint_node', anonymous=True)

    # flight mode object
    modes = fcuModes ()

    # controller object
    cnt = Controller ()

    # ROS loop rate
    rate = rospy.Rate (5.0)

    # Subscribe to drone state
    rospy.Subscriber ('mavros/state', State, cnt.stateCb)
    rospy.Subscriber ("/mavros/global_position/raw/fix", NavSatFix, globalPositionCallback)
    rospy.Subscriber ('mavros/local_position/pose', PoseStamped, localPositionCallback)
    rospy.Subscriber ('radius', Float64, image_callback)
    rospy.Subscriber ('konum', camera, cam_konum_callback)
    rospy.Subscriber ('mavros/altitude', Altitude, amslcallback)
    rospy.Subscriber ('mavros/altitude', Altitude, altitude_callback)

    # Setpoint publisher
    sp_glob_pub = rospy.Publisher ('mavros/setpoint_raw/global', GlobalPositionTarget, queue_size=1)
    sp_pub = rospy.Publisher ('mavros/setpoint_raw/local', PositionTarget, queue_size=1)

    # Make sure the drone is armed
    while not cnt.state.armed:
        modes.setArm ()
        rate.sleep ()
    print ("home amsl altitude", amsl)
    modes.setTakeoff ()
    rospy.sleep (10)
    # We need to send few setpoint messages, then activate OFFBOARD mode, to take effect
    print ("MAIN: SET OFFBOARD")
    # activate OFFBOARD mode
    modes.setOffboardMode ()
    print ("takeoff amsl altitude", amsl)
    waypointmove ()


if __name__ == '__main__':
    try:

        main ()
    except rospy.ROSInterruptException:
        pass