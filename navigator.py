#!/usr/bin/env python

import rospy
import string 
import numpy as np
import numpy.linalg as npl
from modeController import Mode

from std_msgs.msg import Int8, Float64
from geometry_msgs.msg import PoseStamped, Point
from basic_controller.msg import Waypoint

WAYPOINT_ALTITUDE = 50
DISTANCE_THRESHOLD = 5

def createPoint(array):
	point = Point()
	point.x = array[0]
	point.y = array[1]
	point.z = WAYPOINT_ALTITUDE
	return point

def distance(p1, p2):
	p1 = np.array([p1.x, p1.y, p1.z])
	p2 = np.array([p2.x, p2.y, p2.z])

	distance = npl.norm(p1 - p2)
	return abs(distance)

class Navigator():
	def __init__(self):
		rospy.init_node("navigator", anonymous=True)

		#Subscribers
		# rospy.Subscriber('/modeController/mode', Int8, self.modeCallback)
		rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.poseCallback)
		rospy.Subscriber('/modeController/mode', Int8, self.modeCallback)

		#Publishers
		self.wp_pub = rospy.Publisher('/navigator/waypoint', Waypoint, queue_size = 10)

		self.pos = Point()
		self.mode = None

		self.waypoints = [([10, 0], True), ([15, 10], True), ([15, 20], False), ([10, 10], True), ([0, 0], True)]
		self.current_wp_ix = 0
		self.current_wp = Waypoint()

	def modeCallback(self, msg):
		#print("inside modeCallback")
		self.mode = Mode(msg.data)

	def poseCallback(self, msg):
		self.pos.x = msg.pose.position.x
		self.pos.y = msg.pose.position.y
		self.pos.z = msg.pose.position.z

	def navigate(self):
		try:
			self.current_wp.position = createPoint(self.waypoints[self.current_wp_ix][0])
			self.current_wp.loiter = self.waypoints[self.current_wp_ix][1]
		except IndexError:
			self.current_wp.position = createPoint([0, 0])
			self.current_wp.loiter = True

		#if we're supposed to be loitering and we are loitering, get ready to go to next waypoint
		if distance(self.pos, self.current_wp.position) < DISTANCE_THRESHOLD and self.mode == Mode.LOITER:
			self.current_wp_ix += 1

		#if we're not supposed to be loitering but we are at the waypoint, get ready to go to next waypoint
		elif distance(self.pos, self.current_wp.position) < DISTANCE_THRESHOLD and self.current_wp.loiter == False:
			self.current_wp_ix += 1

	def publish(self):
		if self.mode == Mode.WAYPOINT:
			self.wp_pub.publish(self.current_wp)


	def run(self):
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			self.navigate()
			self.publish()
			rate.sleep()

if __name__ == '__main__':
	navigator = Navigator()
	navigator.run()