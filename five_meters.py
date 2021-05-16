#!/usr/bin/env python
# ROS python API
import rospy

# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped, Twist
from sensor_msgs.msg import NavSatFix
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from std_msgs.msg import String, Float64
from decimal import *
from math import radians, cos, sin, asin, sqrt

# Message publisher for haversine
velocity_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=10)
msg1 = Twist()

# Current Position
latitude = 47.397742
longitude = 8.5455936
altitude = 0.0


# Position before Move function execute
previous_latitude = 47.397742
previous_longitude = 8.5455936
previous_altitude = 0.0

def globalPositionCallback(globalPositionCallback):
    global latitude
    global longitude
    latitude = globalPositionCallback.latitude
    longitude = globalPositionCallback.longitude
# Flight modes class
# Flight modes are activated using ROS services
class fcuModes:
    def __init__(self):
        pass

    def setTakeoff(self):
        rospy.wait_for_service('mavros/cmd/takeoff')
  	try:
    		takeoffService = rospy.ServiceProxy(
        '/mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
    		takeoffService(altitude=10, latitude=47.397742,
                   longitude=8.5455936, min_pitch=0, yaw=0)
  	except rospy.ServiceException, e:
   	  print "Service takeoff call failed: %s" % e

    def setArm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            print("Waiting for arming...")
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException, e:
            print ("Service arming call failed: %s"%e)

    def setDisarm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            print("Waiting for disarming...")
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(False)
        except rospy.ServiceException, e:
            print ("Service disarming call failed: %s"%e)

    def setStabilizedMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            print("It's stabilazed mode!")
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='STABILIZED')
        except rospy.ServiceException, e:
            print ("service set_mode call failed: %s. Stabilized Mode could not be set."%e)

    def setOffboardMode(self):
        rospy.wait_for_service('/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
            response = flightModeService(custom_mode='OFFBOARD')
            return response.mode_sent
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Offboard Mode could not be set."%e
            return False

    def setAltitudeMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='ALTCTL')
        except rospy.ServiceException, e:
            print ("service set_mode call failed: %s. Altitude Mode could not be set."%e)

    def setPositionMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='POSCTL')
        except rospy.ServiceException, e:
            print ("service set_mode call failed: %s. Position Mode could not be set."%e)

    def setLandMode(self):
	global pos_mode
	pos_mode = False
	rospy.wait_for_service('/mavros/cmd/land')
	try:
		landService = rospy.ServiceProxy('/mavros/cmd/land', mavros_msgs.srv.CommandTOL)
		isLanding = landService(altitude = 0)
	except rospy.ServiceException, e:
		print "service land call failed: %s. The vehicle cannot land "%e

class Controller:
    # initialization method
    def __init__(self):
        # Drone state
        self.state = State() #using that msg for send few setpoint messages, then activate OFFBOARD mode, to take effect
        # Instantiate a setpoints message
        self.sp = PositionTarget()
        # set the flag to use position setpoints and yaw angle

    ## Drone State callback
    def stateCb(self, msg):
        self.state = msg

def haversine():
        global previous_longitude
  	global previous_latitude
  	global longitude
  	global latitude

	print("calculating distance")

  	lon1, lat1, lon2, lat2 = previous_longitude, previous_latitude, longitude, latitude
	print("longitude", longitude)
	print("latitude", latitude)
  	# lat, long in radians
	lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])

   	# difference lat, long
	dlon = abs(lon2 - lon1) 
 	dlat = abs(lat2 - lat1)

    ## haversine formula 
	a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
	c = 2 * asin(sqrt(a)) 
	r = 6371 # Radius of earth in kilometers
	print("distance calculated:", c*r*1000)
	return Decimal(c * r*1000 )
 




def moveX(distance, speed):
	# for print statements, debugging, delete later
	global latitude
	global longitude
	global previous_latitude
	global previous_longitude
	# end of debug

	msg1.linear.x = speed
        rate = rospy.Rate(20.0)


	while not rospy.is_shutdown():
		print("going to position")
		# compute the distance with haversine formula
		# home_latitude = starting latitude before movement
		# home longitude = starting longitude before movement
		# latitude = current latitude during move
		# longitude = current longitude during move
		haversine_distance = haversine()
		# break loop if desired distance covered 
		if haversine_distance > distance:
			break

		velocity_pub.publish(msg1)

		# This line might cause error because function have no access to rate object in main. 
		# Should i move into global space or pass as argument
		rate.sleep()

		# TODO:: lower speed while getting closer to the final position
		# TODO:: check if rospy.sleep() or rate.sleep() is better
		# TODO:: is creating rate object necessary?

	print("Latitude: {:.7f} ,Longitude: {:.7f}\nDistance Covered: {:.7f}".format(latitude, longitude, haversine_distance))

	# Tell drone to stop its movement
	# Not sure if this works. Need to test on gazebo
	msg1.linear.x = 0.
	msg1.linear.y = 0.
	msg1.linear.z = 0.
	for i in range(100):
		velocity_pub.publish(msg1)

	# DEBUG
	print("Speed X: {} ,Speed Y: {} ,Speed Z: {} ".format(msg1.linear.x, msg1.linear.y, msg1.linear.z))


	# after reaching desired position, set home_latitude, home_longitude to current position value
	previous_latitude = latitude
	previous_longitude = longitude





# Main function
def main():

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


    # Setpoint publisher    
    sp_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)
    
    rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, globalPositionCallback)


    # Make sure the drone is armed
    while not cnt.state.armed:
        modes.setArm()
        rate.sleep()
	
    modes.setTakeoff()
    rospy.sleep(8)

    # We need to send few setpoint messages, then activate OFFBOARD mode, to take effect
    k=0
    while k<10:
        sp_pub.publish(cnt.sp)
        rate.sleep()
        k = k + 1

    print("MAIN: SET OFFBOARD")
    # activate OFFBOARD mode
    modes.setOffboardMode()
    moveX(5, 2)
    modes.setLandMode()
if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass

