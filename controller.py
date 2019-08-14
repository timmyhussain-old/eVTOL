#!/usr/bin/env python

import rospy
import numpy as np
import numpy.linalg as npl
from mavros_msgs.srv import CommandLong, CommandInt, CommandTOL, WaypointPush
import mavros
from mavros import command, mission
# import mavros_msgs.srv
from modeController import Mode, TypeMasks, MavVtolState
	
from std_msgs.msg import Int8, Float64
from geometry_msgs.msg import PoseStamped, Point, Vector3, TwistStamped, AccelWithCovarianceStamped, Quaternion
from mavros_msgs.msg import PositionTarget, State, Thrust, AttitudeTarget
from mavros_msgs.msg import Waypoint as mavros_wp
from sensor_msgs.msg import Imu
from basic_controller.msg import Waypoint

ACCEPTANCE_RADIUS = 1
#cross track error
def xte(p1, p2, p3):
	'''returns cross track error given two waypoints and position of aircraft'''
	p1 = np.array([p1.x, p1.y])
	p2 = np.array([p2.x, p2.y])
	p3 = np.array([p3.x, p3.y])

	xte = np.cross(p2-p1,p3-p1)/norm(p2-p1)
	return xte 

def getQuaternion(angle, axis):
	'''returns Quaternion given an angle and axis about which to rotate'''
	if axis == 'x':
		arr = np.array([1, 1, 0, 0])
	elif axis == 'y':
		arr = np.array([1, 0, 1, 0])
	elif axis == 'z':
		arr = np.array([1, 0, 0, 1])

	rad = 0.5*angle *np.pi/180
	qtrnion = arr*(np.array([np.cos(rad), np.sin(rad), np.sin(rad), np.sin(rad)]))

	# qternion = arr * qternion

	quaternion = Quaternion()

	quaternion.w = qtrnion[0]
	quaternion.x = qtrnion[1]
	quaternion.y = qtrnion[2]
	quaternion.z = qtrnion[3]
	return quaternion

def pointcontroller(current_pos, desired_pos, current_vel, current_acc): 
	'''returns command velocity Vector given vehicle current position, velocity, acceleration and desired position'''
	k_p = 1.0
	k_d = 0.8
	k_i = 0.2

	des_pos = np.array([desired_pos.x, desired_pos.y, desired_pos.z])
	cur_pos = np.array([current_pos.x, current_pos.y, current_pos.z])
	cur_vel = np.array([current_vel.x, current_vel.y, current_vel.z])
	cur_acc = np.array([current_acc.x, current_acc.y, current_acc.z])
	
	error = des_pos-cur_pos


	cmd_vel = k_p * error - k_i * cur_vel - k_d * cur_acc

	ret = Vector3()
	ret.x = cmd_vel[0]
	ret.y = cmd_vel[1]
	ret.z = cmd_vel[2]

	return ret

class Controller():
	def __init__(self):
		#initialise a node
		rospy.init_node("controller", anonymous=True)


		#publishers
		self.cmd_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
		self.cmd_att = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)

		#subscribers
		rospy.Subscriber('/modeController/mode', Int8, self.modeCallback) 
		rospy.Subscriber('/mavros/local_position/velocity_body', TwistStamped, self.velocityCallback)
		rospy.Subscriber('/mavros/local_position/accel', AccelWithCovarianceStamped, self.accelCallback)
		rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.poseCallback)
		rospy.Subscriber('/userInput/position', Vector3, self.userinputCallback)
		rospy.Subscriber('/navigator/waypoint', Waypoint, self.waypointCallback)
		rospy.Subscriber('/mavros/imu/data', Imu, self.imuCallback)
		rospy.Subscriber('/mavros/global_position/compass_hdg', Float64, self.headingCallback)

		#services
		mavros.set_namespace()
		# self.cmd_long = command._get_proxy('command', CommandLong)
		self.cmd_long = None
		# self.cmd_int = command._get_proxy('command_int', CommandInt)
		# self.cmd_land = command._get_proxy('land', CommandTOL)

		#waypoint services
		self.push = rospy.ServiceProxy(mavros.get_topic('mission', 'push'), WaypointPush)
		self.mavros_wp = mavros_wp()
		self.mavros_wp.frame = 1

		self.home = Point()
		self.home.x = 0
		self.home.y = 0
		self.home.z = 0

		self.mode = None
		self.takeoff = None
		self.loiter = None
		self.idle = None
		self.pos = Point()
		self.vel = Vector3()
		self.accel = Vector3()
		self.orientation = Quaternion()
		self.heading = Float64()

		
		self.cmd = PositionTarget()
		self.cmd.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
		# self.cmd.type_mask = 0b0000111111000111		#mask velocity
		self.current_wp = Waypoint()

		self.transition_flag = True
		self.landing_flag = True
		self.waypoint_flag = True

		'''
		self.cmd_vel = Vector3()
		self.cmd_vel.x = 0	#E
		self.cmd_vel.y = 0	#N
		self.cmd_vel.z = 0	#U

		self.cmd_pos = Vector3()
		self.cmd_pos.x = 0
		self.cmd_pos.y = 0
		self.cmd_pos.z = 0
		'''

	def modeCallback(self, msg):
		'''get mode as published by modeController'''
		self.mode = Mode(msg.data)

	def poseCallback(self, msg):
		'''get pose as published by mavros node'''
		self.pos.x = msg.pose.position.x
		self.pos.y = msg.pose.position.y
		self.pos.z = msg.pose.position.z

	def velocityCallback(self, msg):
		'''get velocity as published by mavros node'''
		self.vel.x = msg.twist.linear.x
		self.vel.y = msg.twist.linear.y
		self.vel.z = msg.twist.linear.z

	def accelCallback(self, msg):
		'''get acceleration as published by mavros node'''
		self.accel.x = msg.accel.linear.x
		self.accel.y = msg.accel.linear.y
		self.accel.z = msg.accel.linear.z

	def imuCallback(self, msg):
		'''get IMU data as published by mavros node'''
		self.orientation.w = msg.orientation.w
		self.orientation.x = msg.orientation.x
		self.orientation.y = msg.orientation.y
		self.orientation.z = msg.orientation.z
		
		# rospy.loginfo("current w: %s\n Current x: %s\nCurrent y: %s\nCurrent z: %s\n", self.orientation.w, self.orientation.x, self.orientation.y, self.orientation.z)

	def headingCallback(self, msg):
		'''get heading as published by mavros node'''
		self.heading.data = msg.data
		
		# rospy.loginfo("current heading: %s\n", msg.data)

	def waypointCallback(self, msg):
		'''get waypoint as published by navigator node'''
		self.current_wp.position = msg.position
		self.current_wp.loiter = msg.loiter

	def userinputCallback(self, msg):
		'''get user input waypoint as published by userinput node'''
		self.cmd_pos.x = msg.x
		self.cmd_pos.y = msg.y
		self.cmd_pos.z = msg.z

	def control(self):
		'''main control loop'''

		if self.mode == Mode.TAKEOFF:
			
			#position control
			self.cmd.type_mask = TypeMasks.MASK_POSITION.value
			desired_pos = Vector3()
			desired_pos.x = desired_pos.y = 0
			desired_pos.z = 10

			self.cmd.header.stamp = rospy.get_rostime()
			self.cmd.position = desired_pos
			self.cmd_pub.publish(self.cmd)


			# self.cmd_pos.x = desired_pos.x
			# self.cmd_pos.y = desired_pos.y
			# self.cmd_pos.z = desired_pos.z

			#velocity control
			'''
			self.cmd.type_mask = 0b0000111111000111 
			self.cmd_vel = pointcontroller(self.pos, self.cmd_pos, self.vel, self.accel)
			'''

			#attitude control: thrust command and no rotation quaternion
			'''
			att = AttitudeTarget()
			att.type_mask = 0b000111
			att.thrust = 0.750
			att.orientation = Quaternion()
			att.orientation.w = 1.0
			att.orientation.x = 0.0
			att.orientation.y = 0.0 
			att.orientation.z = 0.0
			self.cmd_att.publish(att)
			'''

		elif self.mode == Mode.TRANSITION_FW:
			self.cmd_long = command._get_proxy('command', CommandLong)
			# rospy.wait_for_service('mavros_msgs/CommandLong')
			
			

			params =	{"command": 3000,
						"param1": MavVtolState.MAV_VTOL_STATE_FW.value,
						"param2": 0,
						"param3": 0,
						"param4": 0,
						"param5": 0, 
						"param6": 0,
						"param7": 0}

			if self.transition_flag == True:	
				try:
					self.cmd_long(**params)
					self.transition_flag = False
					self.cmd_long.close()
				except rospy.ServiceException as exc:
					print("Service did not process request: "+str(exc))		

		elif self.mode == Mode.TRANSITION_MC:
			self.cmd_long = command._get_proxy('command', CommandLong)
			# rospy.wait_for_service('mavros_msgs/CommandLong')
			
			

			params =	{"command": 3000,
						"param1": MavVtolState.MAV_VTOL_STATE_MC.value,
						"param2": 0,
						"param3": 0,
						"param4": 0,
						"param5": 0, 
						"param6": 0,
						"param7": 0}

			if self.transition_flag == False:	
				try:
					self.cmd_long(**params)
					self.transition_flag = True
					self.cmd_long.close()
				except rospy.ServiceException as exc:
					print("Service did not process request: "+str(exc))		

		elif self.mode == Mode.USER:
			self.cmd.type_mask = 0b0000111111000111
			self.cmd_vel = pointcontroller(self.pos, self.cmd_pos, self.vel, self.accel)

		#quadcopter mode waypoint
		elif self.mode == Mode.WAYPOINT:
			self.cmd.header.stamp = rospy.get_rostime()
			self.cmd.type_mask = TypeMasks.MASK_POSITION.value
			self.cmd.position = self.current_wp.position
			self.cmd_pub.publish(self.cmd) 
		



		#quadcopter mode loiter
		elif self.mode == Mode.LOITER:
			# self.cmd.header.stamp = rospy.get_rostime()
			# self.cmd.type_mask = TypeMasks.MASK_POSITION.value
			# self.cmd.position = self.current_wp.position
			# self.cmd_pub.publish(self.cmd) 

			quaternion = getQuaternion(180, 'x')

			att = AttitudeTarget()
			att.header.stamp = rospy.get_rostime()
			att.type_mask = 0b000111
			att.thrust = 0.8
			att.orientation = Quaternion()
			att.orientation.w = quaternion.w
			att.orientation.x = quaternion.x
			att.orientation.y = quaternion.y
			att.orientation.z = quaternion.z
			self.cmd_att.publish(att)

		#VTOL mode waypoint
		# elif self.mode == Mode.WAYPOINT:
		# 	#get heading
		# 	# orientation saved in self.orientation

		# 	#get bearing from current position to intended position
		# 	# waypoint_1 = Point()
		# 	# waypoint_1.x = -30
		# 	# waypoint_1.y = -10
		# 	# waypoint_1.z = 30

		# 	# waypoint_2 = Point()
		# 	# waypoint_2.x = 20
		# 	# waypoint_2.y = 20
		# 	# waypoint_2.z = 10


		# 	# self.cmd.type_mask = TypeMasks.MASK_LOITER_POSITION.value
		# 	# self.cmd.position = self.current_wp.position
		# 	# self.cmd_pub.publish(self.cmd)
			
			
		# 	#thrust command:
		# 	del_h = self.pos.z - self.current_wp.position.z
		# 	if abs(del_h) < 10:
		# 		#thrust gets us to the right altitude, quaternion directs us to the right x-y location
		# 		thr = 0.76 + 0.036*(14.4 - self.vel.x)

		# 		y = np.array([self.current_wp.position.y - self.pos.y])
		# 		x = np.array([self.current_wp.position.x - self.pos.x])
		# 		angle_to_point = np.arctan2(y, x)[0] * 180/np.pi

		# 		#get current heading
		# 		angle_control_input = angle_to_point - self.heading.data

		# 		quaternion = getQuaternion(angle_control_input, 'x')

		# 		att = AttitudeTarget()
		# 		att.type_mask = 0b000111
		# 		att.thrust = thr
		# 		att.orientation = Quaternion()
		# 		att.orientation.w = quaternion.w
		# 		att.orientation.x = quaternion.x
		# 		att.orientation.y = quaternion.y
		# 		att.orientation.z = quaternion.z
		# 		self.cmd_att.publish(att)
		# 		# xte = xte() cross track error
		# 		# pass
		# 	else:
		# 		#get pitch control and get us to the right altitude roughly
		# 		z = np.array([self.current_wp.position.z - self.pos.z])
		# 		x = np.array([self.current_wp.position.x - self.pos.x])
		# 		pitch_to_point = np.arctan2(z, x)[0] *180/np.pi
		# 		print(pitch_to_point)
		# 		if pitch_to_point > 45:
		# 			pitch_to_point = 45
		# 		elif pitch_to_point < -45:
		# 			pitch_to_point = -45
		# 		quaternion = getQuaternion(pitch_to_point, 'x')
		# 		# quaternion = Quaternion()
		# 		# quaternion.w = qtrn.w - self.orientation.w
		# 		# quaternion.x = qtrn.x - self.orientation.x
		# 		# quaternion.y = qtrn.y - self.orientation.y
		# 		# quaternion.z = qtrn.z - self.orientation.z
				

		# 		att = AttitudeTarget()
		# 		att.type_mask = 0b000111
		# 		att.orientation = Quaternion()
		# 		att.thrust = 0.8
		# 		att.orientation.w = quaternion.w
		# 		att.orientation.x = quaternion.x
		# 		att.orientation.y = quaternion.y
		# 		att.orientation.z = quaternion.z
		# 		self.cmd_att.publish(att)
		# 		#pass
			

			#read in waypoint --> happening in self.waypointCallback

			#navigate to that position using either position controller or mav_cmd_nav_waypoint
			


			# self.mavros_wp.command = 16

			# self.mavros_wp.is_current = True
			# self.mavros_wp.autocontinue = not (self.current_wp.loiter.data)

			# self.mavros_wp.param1 = 5
			# self.mavros_wp.param2 = ACCEPTANCE_RADIUS
			# self.mavros_wp.param3 = 5
			# self.mavros_wp.param4 = 0

			# self.mavros_wp.x_lat = self.current_wp.position.x
			# self.mavros_wp.y_long = self.current_wp.position.y
			# self.mavros_wp.z_alt = self.current_wp.position.z

			# rospy.wait_for_service(mavros.get_topic('mission', 'push'))

			# params = 	{"start_index": 0,
			# 			"waypoints": [self.mavros_wp]}
			# if self.waypoint_flag == True:
			# 	try:
			# 		self.push(**params)
			# 		self.waypoint_flag = False
			# 	except rospy.ServiceException as exc:
			# 		print("Service did not process request: "+str(exc))	


		elif self.mode == Mode.LAND:
			params =	{"min_pitch": 0,
						"yaw": 0,
						"latitude": self.home.x,
						"longitude": self.home.y,
						"altitude": 0}
			
			if self.landing_flag == True:							
				try:
					self.cmd_land(**params)
					self.landing_flag = False
				except rospy.ServiceException as exc:
					print("Service did not process request: "+str(exc))	

	def publish(self):
		
		self.cmd.header.stamp = rospy.get_rostime()
		# self.cmd.velocity = self.cmd_vel
		# self.cmd.position = self.cmd_pos
		self.cmd_pub.publish(self.cmd)
		

	def run(self):
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			self.control()
			# self.publish()
			rate.sleep()

if __name__ == '__main__':
	controller = Controller()
	controller.run()
