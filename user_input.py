#!/usr/bin/env python

import rospy
import string 
from geometry_msgs.msg import Vector3

class userInput():
	def __init__(self):
		#initialise node
		rospy.init_node("userInput", anonymous = True)

		#subscribers
		#rospy.Subscriber()

		#publisher
		self.cmd = Vector3()
		self.cmd_pub = rospy.Publisher('/userInput/position', Vector3, queue_size = 10)

	def read_input(self):
		try:
			inp = input("input position in the form [x y z]: \n")
			x = str(inp).split(' ')
			#print(inp)
			arr = [float(a) for a in x]
			self.cmd.x = arr[0]
			self.cmd.y = arr[1]
			self.cmd.z = arr[2]
		except SyntaxError:
			pass
		except NameError:
			pass
		except IndexError:
			pass
		finally:
			pass



	def publish(self):
		self.cmd_pub.publish(self.cmd)

	def run(self):
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			self.read_input()
			self.publish()

if __name__ == '__main__':
	user_input = userInput()
	user_input.run()