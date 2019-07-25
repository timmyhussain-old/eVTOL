#!/usr/bin/env python

"""
Spyder Editor

This is a temporary script file.
"""

import rospy 
from enum import Enum
from std_msgs.msg import Int8, Bool, Float32
from geometry_msgs.msg import Pose, PoseStamped, Vector3
from sensor_msgs.msg import BatteryState
from mavros_msgs.msg import State

TAKEOFF_ALT_THRESHOLD = 9          # Altitude at which we have finished take-off
RETURN_BATTERY_THRESHOLD = 0.40     # battery threshold for returning home
HOME_POS_THRESH = 5.0               # Position error Threshold for determining once we're home
IDLE_TIME = 5.0                     # sit in idle for this long before taking off
MAX_BATTERY_CHARGE = 4400.          # Maximum battery charge in Mah

class Mode(Enum):
    IDLE = 1
    TAKEOFF = 2
    TRANSITION_FW = 3
    HOLD = 4
    LAND = 5
    USER = 6

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
        
        self.transition_start = None

        # publishers
        self.mode_publisher = rospy.Publisher('/modeController/mode', Int8, queue_size=10)
        self.home_publisher = rospy.Publisher('/modeController/home', Pose, queue_size=10)
        
        #subscribers
        rospy.Subscriber('/mavros/state', State, self.stateCallback)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.poseCallback)       
        rospy.Subscriber('/mavros/battery',BatteryState,self.batteryCallback)
        
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
        
    def determineMode(self):
       # if self.
        
        if self.mode == Mode.IDLE:
            if self.drone_mode == "OFFBOARD" and self.hasInitialized():
                self.mode = Mode.TAKEOFF
        
        elif self.mode == Mode.TAKEOFF and self.hasTakenOff():
            #self.mode = Mode.TRANSITION_FW
            #self.transition_start = rospy.get_rostime()
            #self.mode = Mode.USER
            pass

        elif self.mode == Mode.TRANSITION_FW:
            now = rospy.get_rostime()
            if now.secs - self.transition_start.secs > 10:
                self.mode = Mode.USER
        
        #elif self.mode = Mode.HOLD:
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
                
                    
                
                
        
        
