#!/usr/bin/env python

"""
Spyder Editor

This is a temporary script file.
"""

import rospy 
from enum import Enum
from std_msgs.msg import Int8, Bool, Float32
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import BatteryState
from mavros_msgs.msg import State

TAKEOFF_ALT_THRESHOLD = 5          # Altitude at which we have finished take-off
RETURN_BATTERY_THRESHOLD = 0.40     # battery threshold for returning home
HOME_POS_THRESH = 5.0               # Position error Threshold for determining once we're home
IDLE_TIME = 5.0                     # sit in idle for this long before taking off
MAX_BATTERY_CHARGE = 4400.          # Maximum battery charge in Mah

class Mode(Enum):
    IDLE = 1
    TAKEOFF = 2
    HOLD = 3
    LAND = 4

class ModeController():
    def __init__(self):
        rospy.init_node("mode_controller", anonymous=True)
        
        self.mode = Mode.IDLE
        self.prev_mode = None
        self.drone_mode = None
        self.battery_level = 0.0
        self.battery_status = None
        self.mission_start_time = None
        
        self.pos = None
        
        self.hold_start = None

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
        
        rospy.loginfo("Current x: %s\n Current y: %s\n Current z: %s\n", x, y, z)

    def batteryCallback(self, msg):
        self.battery_status = msg
        self.battery_level = msg.percentage
        
    def determineMode(self):
       # if self.
        
        if self.mode == Mode.IDLE:
            if self.drone_mode == "OFFBOARD" and self.hasInitialized():
                self.mode = Mode.TAKEOFF
        
        elif self.mode == Mode.TAKEOFF and self.hasTakenOff():
            self.mode = Mode.HOLD
            self.hold_start = rospy.get_rostime()
        
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
                
                    
                
                
        
        