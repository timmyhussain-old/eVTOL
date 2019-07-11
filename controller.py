#!/usr/bin/env python

import rospy
import numpy
from modeController import Mode

from std_msgs.msg import Int8, Float64
from geometry_msgs.msg import PoseStamped, Point, Vector3, TwistStamped
from mavros_msgs.msg import PositionTarget, State

class Controller():
	def __init__(self):
		#initialise a node
		rospy.init_node("controller", anonymous=True)


		#publishers
		self.cmd_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)


		#subscribers
		rospy.Subscriber('/modeController/mode', Int8, self.modeCallback)

		self.mode = None
		self.takeoff = None
		self.loiter = None
		self.idle = None
		self.pos = None
		self.vel = None

		self.cmd = PositionTarget()
		self.cmd.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
		self.cmd.type_mask = 0b0000111111000111		#mask velocity

		self.cmd_vel = Vector3()
		self.cmd_vel.x = 0	#E
		self.cmd_vel.y = 0	#N
		self.cmd_vel.z = 0	#U

		self.cmd_pos = Vector3()
		self.cmd_pos.x = 0
		self.cmd_pos.y = 0
		self.cmd_pos.z = 5

	def modeCallback(self, msg):
		#print("inside modeCallback")
		self.mode = Mode(msg.data)
		#ros.loginfo("Current mode: %s", self.mode.data)

	def control(self):
		#print(self.mode)
		if self.mode == Mode.TAKEOFF:
			self.cmd.type_mask = 0b0001110111111000	#takeoff type mask

		if self.mode == Mode.HOLD:
			self.cmd.type_mask = 0b0100111111111100

	def publish(self):
		self.cmd.header.stamp = rospy.get_rostime()
		self.cmd.velocity = self.cmd_vel
		self.cmd.position = self.cmd_pos
		self.cmd_pub.publish(self.cmd)

	def run(self):
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			self.control()
			self.publish()
			rate.sleep()

if __name__ == '__main__':
	controller = Controller()
	controller.run()