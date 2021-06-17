#!/usr/bin/env python
# -*- coding: utf-8 -*-
# ROS python API
import rospy, mavros,math,cv2
# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped, Twist,TwistStamped
from sensor_msgs.msg import NavSatFix,Image
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from std_msgs.msg import String, Float64,Int64
from decimal import *
from cv_bridge import CvBridge, CvBridgeError
# Message publisher for haversine
velocity_pub =rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=10)
msg1 = PositionTarget()
# Current Position
latitude = 0
longitude = 0
altitude=0
local_x = 0
local_y = 0
# Position before Move function execute
previous_latitude = 0
previous_longitude = 0
previous_altitude = 0.0
global sp_pub,sp_glob_pub

mavros.set_namespace() #for global callback
def globalPositionCallback(globalPositionCallback):
    global latitude,longitude,altitude
    latitude = globalPositionCallback.latitude
    longitude = globalPositionCallback.longitude
    altitude = globalPositionCallback.altitude

amsl=0
def amslcallback(data):
    global amsl
    amsl= float("{0:.1f}".format(data.amsl))


def localPositionCallback(localPositionCallback):
    global local_x, local_y, local_w
    local_x = localPositionCallback.pose.position.x
    local_y = localPositionCallback.pose.position.y

def glob_pos_pub (wp_lat,wp_long,wp_alt):
    cnt=Controller()
    global altitude,latitude,longitude,sp_glob_pub,amsl
    cnt.sp_glob.latitude = wp_lat
    cnt.sp_glob.longitude = wp_long
    cnt.sp_glob.altitude = amsl+wp_alt
    rate = rospy.Rate(20.0)

    while not rospy.is_shutdown():
        rate.sleep()
        sp_glob_pub.publish(cnt.sp_glob)
        latitude = float("{0:.6f}".format(latitude))
        longitude = float("{0:.6f}".format(longitude))
        print(latitude,wp_lat,longitude,wp_long,cnt.sp_glob.altitude,amsl)
        #print("poselanıyor")
        #print(longitude,latitude,altitude,cnt.sp_glob.altitude)
        if(latitude-0.000001) < wp_lat <(latitude+0.000001) and (longitude-0.000002)< wp_long <(longitude+0.000002) and (amsl-0.5 )< cnt.sp_glob.altitude < (amsl +0.5):
            print("Konuma gidildi.")
            break

# cv2 bridge
bridge = CvBridge()
cv_image = ""
last_red_latitude = 0
last_red_longitude = 0
red_latitude = 0
red_longitude = 0
pre_radius=0


def image_callback(radius):
	global red_latitude,red_longitude,latitude,longitude,pre_radius
	pre_radius= radius
	red_latitude=latitude
	red_longitude=longitude
	print("************************GOT DATA***************************",radius,red_latitude,red_longitude)
	rate = rospy.Rate(20.0)
	rate.sleep()

konum=10
def cam_konum_callback(data):
    global konum
    konum=int (data.data)
    rate = rospy.Rate(20.0)
    rate.sleep()

# Flight modes class
class fcuModes:
    def __init__(self):
        pass

    def setTakeoff(self):
        global longitude, latitude
        rospy.wait_for_service('mavros/cmd/takeoff')
        try:
            takeoffService = rospy.ServiceProxy(
                '/mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
            takeoffService(altitude=0, latitude=latitude,
                           longitude=longitude, min_pitch=0, yaw=0)
        except rospy.ServiceException as e:
            print("Service takeoff call failed: %s" % e)

    def setArm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            print("Waiting for arming...")
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException, e:
            print("Service arming call failed: %s" % e)

    def setDisarm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            print("Waiting for disarming...")
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(False)
        except rospy.ServiceException, e:
            print("Service disarming call failed: %s" % e)

    def setStabilizedMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            print("It's stabilazed mode!")
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='STABILIZED')
        except rospy.ServiceException, e:
            print("service set_mode call failed: %s. Stabilized Mode could not be set." % e)

    def setOffboardMode(self):
        global sp_glob_pub
        rospy.wait_for_service('/mavros/set_mode')
        cnt = Controller()
        rate = rospy.Rate(20.0)
        k = 0
        while k < 12:
            sp_glob_pub.publish(cnt.sp_glob)
            rate.sleep()
            k = k + 1
            rospy.wait_for_service('/mavros/set_mode')

        try:
            flightModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
            response = flightModeService(custom_mode='OFFBOARD')
            return response.mode_sent
        except rospy.ServiceException as e:
            print ("service set_mode call failed: %s. Offboard Mode could not be set." % e)
            return False


    def setAltitudeMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='ALTCTL')
        except rospy.ServiceException, e:
            print("service set_mode call failed: %s. Altitude Mode could not be set." % e)

    def setLoiterMode(self):
        rospy.wait_for_service('/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
            isModeChanged = flightModeService(custom_mode='AUTO.LOITER')  # return true or false
        except rospy.ServiceException as e:
            print(
                "service set_mode call failed: %s. AUTO.LOITER Mode could not be set. Check that GPS is enabled %s" % e)

    def setPositionMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='POSCTL')
        except rospy.ServiceException, e:
            print("service set_mode call failed: %s. Position Mode could not be set." % e)

    def setLandMode(self):
        global pos_mode
        pos_mode = False
        rospy.wait_for_service('/mavros/cmd/land')
        try:
            landService = rospy.ServiceProxy('/mavros/cmd/land', mavros_msgs.srv.CommandTOL)
            isLanding = landService(altitude=0)
        except rospy.ServiceException, e:
            print
            "service land call failed: %s. The vehicle cannot land " % e


class Controller:

    # initialization method
    def __init__(self):
        global local_x, local_y, local_w
        # Drone state
        self.state = State()  # using that msg for send few setpoint messages, then activate OFFBOARD mode, to take effect
        # Instantiate a setpoints message
        self.sp_glob = GlobalPositionTarget()
        self.sp_glob.type_mask = int('010111111000', 2)
        self.sp_glob.coordinate_frame = 6  # FRAME_GLOBAL_INT
        self.sp_glob.latitude = 0
        self.sp_glob.longitude = 0
        self.sp_glob.altitude = 0
        self.level = PoseStamped()
        self.sp = PositionTarget()

        self.local_position_publisher = rospy.Publisher(mavros.get_topic('setpoint_position', 'local'), PoseStamped, queue_size=10)
    # set the flag to use position setpoints and yaw angle
    # self.wp.position.z = self.ALT_SP
        self.local_pos = Point(0, 0, 0)
        self.sp_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)
        self.rate = rospy.Rate(20.0)

## Drone State callback
    def stateCb(self, msg):
        self.state = msg

    def updateSp(self):
        self.level.pose.position.x = local_x
        self.level.pose.position.y = local_y

def movingcenter():
    global konum,msg1,velocity_pub
    while 1:
        msg1.header.stamp = rospy.get_rostime ()
        msg1.header.frame_id = "world"
        msg1.coordinate_frame = 8
        msg1.type_mask = int ('011111000111', 2)
        msg1.velocity.z = 0
        print (konum)
        if konum ==1:
            msg1.velocity.x = 0.5
            msg1.velocity.y = -0.5
            while not rospy.is_shutdown ():
                velocity_pub.publish(msg1)
                if not konum ==1:
                    break
        elif konum ==2:
            msg1.velocity.x = 0
            msg1.velocity.y= -0.5
            while not rospy.is_shutdown ():
                velocity_pub.publish(msg1)
                if not konum ==2:
                    break
        elif konum ==3:
            msg1.velocity.x = -0.5
            msg1.velocity.y = -0.5
            while not rospy.is_shutdown ():
                velocity_pub.publish(msg1)
                if not konum ==3:
                    break

        elif konum ==4:
            msg1.velocity.x = -0.5
            msg1.velocity.y = 0
            while not rospy.is_shutdown ():
                velocity_pub.publish (msg1)
                if not konum ==4:
                    break

        elif konum ==5:
            msg1.velocity.x= -0.5
            msg1.velocity.y= 0.5
            while not rospy.is_shutdown ():
                velocity_pub.publish(msg1)
                if not konum ==5:
                    break

        elif konum ==6:
            msg1.velocity.x = 0
            msg1.velocity.y = 0.5
            while not rospy.is_shutdown ():
                velocity_pub.publish(msg1)
                if not konum ==6:
                    break

        elif konum ==7:
            msg1.velocity.x = 0.5
            msg1.velocity.y = 0.5
            while not rospy.is_shutdown ():
                velocity_pub.publish(msg1)
                if not konum ==7:
                    break

        elif konum ==8:
            msg1.velocity.x = 0.5
            msg1.velocity.y = 0
            while not rospy.is_shutdown ():
                velocity_pub.publish (msg1)
                if not konum == 8:
                    break


        elif konum ==0:
            msg1.velocity.x = 0
            msg1.velocity.y = 0
            while not rospy.is_shutdown ():
                velocity_pub.publish(msg1)
                if not konum ==0:
                    break

            break

def waypointmove():
    global red_longitude, red_latitude
    modes = fcuModes()
  #  glob_pos_pub( 41.090322,28.617505,0)
 #   glob_pos_pub( red_latitude,red_longitude,0) #kırmızıya git
    movingcenter () #kırmızıyı ortala
 #   print(amsl)
#    glob_pos_pub( red_latitude,red_longitude,-3) #3 metreye alçal
   # print(amsl)
  #  modes.setLoiterMode()
 #   print("3 metreye alçaldı")
#    rospy.sleep(5)
   # modes.setOffboardMode()
  #  glob_pos_pub( red_latitude,red_longitude,3) #7 metreye yüksel
 #   print("7 metreye yükseldi")
#    glob_pos_pub( 41.090322,28.617505,0)
    modes.setLandMode()
# Main function
def main():
    global sp_pub,sp_glob_pub,amsl
    # initiate node
    rospy.init_node('setpoint_node', anonymous=True)

    # flight mode object
    modes = fcuModes()

    # controller object
    cnt = Controller()

    # ROS loop rate
    rate = rospy.Rate(20.0)

    # Subscribe to drone state
    rospy.Subscriber('mavros/state', State, cnt.stateCb)
    rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, globalPositionCallback)
    rospy.Subscriber('mavros/local_position/pose', PoseStamped, localPositionCallback)
    rospy.Subscriber('radius', Float64, image_callback)
    rospy.Subscriber('konum', Int64, cam_konum_callback)
    rospy.Subscriber('mavros/altitude',Altitude,amslcallback)

    # Setpoint publisher
    sp_glob_pub=rospy.Publisher('mavros/setpoint_raw/global', GlobalPositionTarget, queue_size=1)
    sp_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)

    # Make sure the drone is armed
    while not cnt.state.armed:
        modes.setArm()
        rate.sleep()
    print("home amsl altitude", amsl)
    modes.setTakeoff()
    rospy.sleep(10)
    # We need to send few setpoint messages, then activate OFFBOARD mode, to take effect
    print("MAIN: SET OFFBOARD")
    # activate OFFBOARD mode
    modes.setOffboardMode()
    print("takeoff amsl altitude", amsl)
    waypointmove()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass