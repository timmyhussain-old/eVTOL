#!/usr/bin/env python

"""
Spyder Editor

This is a temporary script file.
"""

import rospy 
import numpy as np
import numpy.linalg as npl
from enum import Enum
from std_msgs.msg import Int8, Bool, Float32
from geometry_msgs.msg import Point, Pose, PoseStamped, Vector3
from sensor_msgs.msg import BatteryState
from mavros_msgs.msg import State
from basic_controller.msg import Waypoint

TAKEOFF_ALT_THRESHOLD = 9          # Altitude at which we have finished take-off
RETURN_BATTERY_THRESHOLD = 0.40     # battery threshold for returning home
HOME_POS_THRESH = 5.0               # Position error Threshold for determining once we're home
IDLE_TIME = 5.0                     # sit in idle for this long before taking off
MAX_BATTERY_CHARGE = 4400.          # Maximum battery charge in Mah
DISTANCE_THRESHOLD = 1

def distance(p1, p2):
    p1 = np.array([p1.x, p1.y, p1.z])
    p2 = np.array([p2.x, p2.y, p2.z])

    distance = npl.norm(p1 - p2)
    return abs(distance)


class Mode(Enum):
    IDLE = 1
    TAKEOFF = 2
    TRANSITION_FW = 3
    LOITER = 4
    WAYPOINT = 5
    LAND = 6
    USER = 7

class TypeMasks(Enum): 
    MASK_POSITION =         0b0000111111111000
    MASK_VELOCITY =         0b0000111111000111
    MASK_TAKEOFF_POSITION = 0b0001111111111000
    MASK_TAKEOFF =          0b0001111111111111
    MASK_LAND_VELOCITY =    0b0010111111000111
    MASK_LAND_POSITION =    0b0010111111111000
    MASK_LAND =             0b0010111111111111
    MASK_LOITER_POSITION =  0b0100111111111000
    MASK_IDLE_POSITION =    0b1000111111111000

class MavVtolState(Enum):
    MAV_VTOL_STATE_UNDEFINED = 0
    MAV_VTOL_STATE_TRANSITION_TO_FW = 1
    MAV_VTOL_STATE_TRANSITION_TO_MC = 2
    MAV_VTOL_STATE_MC = 3
    MAV_VTOL_STATE_FW = 4

class ModeController():
    def __init__(self):
        rospy.init_node("modeController", anonymous=True)
        
        self.mode = Mode.TAKEOFF
        self.prev_mode = None
        self.drone_mode = None
        self.battery_level = 0.0
        self.battery_status = None
        self.mission_start_time = None
        
        self.pos = Vector3()
        self.current_wp = Waypoint()
        
        self.transition_start = None
        self.loiter_start = None

        # publishers
        self.mode_publisher = rospy.Publisher('/modeController/mode', Int8, queue_size=10)
        self.home_publisher = rospy.Publisher('/modeController/home', Pose, queue_size=10)
        
        #subscribers
        rospy.Subscriber('/mavros/state', State, self.stateCallback)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.poseCallback)       
        rospy.Subscriber('/mavros/battery',BatteryState,self.batteryCallback)
        rospy.Subscriber('/navigator/waypoint', Waypoint, self.waypointCallback)
        
    def hasInitialized(self):
        check1 = self.battery_status != None
        check2 = self.battery_level > RETURN_BATTERY_THRESHOLD
        return check1 and check2
    
    def hasTakenOff(self):
        return self.pos.z > TAKEOFF_ALT_THRESHOLD
    
    def stateCallback(self, msg):
        #the message contains the current mode of the drone
        self.drone_mode = msg.mode
        
    def poseCallback(self, msg):
        pos = msg.pose.position
        x = pos.x
        y = pos.y
        z = pos.z
        
        self.pos = msg.pose.position
        self.pos.x = x
        self.pos.y = y
        self.pos.z = z
        
        #rospy.loginfo("Current x: %s\n Current y: %s\n Current z: %s\n", x, y, z)

    def batteryCallback(self, msg):
        self.battery_status = msg
        self.battery_level = msg.percentage

    def waypointCallback(self, msg):
        self.current_wp.position = msg.position
        self.current_wp.loiter = msg.loiter
        
    def determineMode(self):
       # if self.
        
        if self.mode == Mode.IDLE:
            if self.drone_mode == "OFFBOARD" and self.hasInitialized():
                self.mode = Mode.TAKEOFF
        
        elif self.mode == Mode.TAKEOFF and self.hasTakenOff():
            # self.mode = Mode.TRANSITION_FW
            # self.transition_start = rospy.get_rostime()
            # self.mode = Mode.USER
            self.mode = Mode.WAYPOINT
            # pass

        elif self.mode == Mode.TRANSITION_FW:
            now = rospy.get_rostime()
            if now.secs - self.transition_start.secs > 3:
                self.mode = Mode.WAYPOINT
        
        elif self.mode == Mode.WAYPOINT:
            if distance(self.pos, self.current_wp.position) < DISTANCE_THRESHOLD and self.current_wp.loiter.data == True:
                self.loiter_start = rospy.get_rostime()
                self.mode = Mode.LOITER

        elif self.mode == Mode.LOITER:
            now = rospy.get_rostime()
            # print(now.secs - self.loiter_start.secs)
            if now.secs - self.loiter_start.secs > 5:
                # print('yes')
                self.mode == Mode.WAYPOINT


    def publish(self):
        msg = Int8()
        msg.data = self.mode.value
        self.mode_publisher.publish(msg)

    def run(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            self.determineMode()
            self.publish()
            rate.sleep()

if __name__ == '__main__':
    mode_controller = ModeController()
    mode_controller.run()
                
                    
                
                
        
        
