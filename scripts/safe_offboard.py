#!/usr/bin/env python

import os
import sys
import paho
import rospy
import mavros

from time 				import sleep
from numpy.linalg		import norm
from math				import pi
from math				import sin
from math				import cos
from math				import sqrt
from math				import atan2

from numpy 				import array 		as numpy_array

from geometry_msgs.msg 	import PoseStamped

from mavros_msgs.msg 	import State
from mavros_msgs.srv 	import SetMode
from mavros_msgs.srv 	import CommandTOL
from mavros_msgs.srv 	import CommandBool

# Define fly time (poses at 20Hz, 800 = 40 seconds)
FLY_TIME = 800

class Offboard_Control() :

	def __init__(self) :
		'''
			Init offboard control mode

			Checks that local position is being published and drone can go to position mode

			Sets publishers and subscribers
		'''
		
		# Init node
		rospy.init_node('Offboard_Control_Node', anonymous=False)

		# Init MAVROS messages
		self.state = State()

		# Init MAVROS services
		self.offb_set_mode = SetMode

		# Current and next pos
		self.pos = PoseStamped()
		self.next_pos = PoseStamped()

		# First subscribe only to local pos published to make sure we can go to position mode
		self.local_pos_sub = rospy.Subscriber('/uav0/mavros/local_position/pose', PoseStamped, self.update_current_pos_cb)

		# Give some time to update current pos
		sleep(1)
		self.home_pos = self.pos

		# Configurable variables
		self.config_vars = {
			"takeoff_height" : 1.23,
			"circle_radius" : 4.2,
		}

		# State variables
		self.state_vars = {
			"armed" : False,
			"land"	: True,
		}
		
		# Rest of publishers and subscribers
		self.local_pos_pub 	 = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
		self.state_sub 		 = rospy.Subscriber("/mavros/state", State, self.update_state_cb)

		# Useful MAVROS services
		self.arming_client 	 = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
		self.landing_client  = rospy.ServiceProxy("/mavros/cmd/land", CommandTOL)
		self.set_mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)

	def _check_ext_positioning(self) :
		'''
			Checks that external positioning is working.

			TODO: Land if pose is delayed from at some of the sources
			TODO: MQTT message
			
		'''

        # Check that local pose has been updated
		if self.pos.pose.position.x == 0 and self.pos.pose.position.y == 0 or self.pos.pose.position.z == 0 :
			rospy.logerr("Could not get local pose. Exitting now")
			return False

		# Save original position
		self.home_pos = self.pos

		return True

	def run(self) :
		'''
			Infinite loop that flies the drone
		'''

		# Set Rospy rate (more than 2Hz required by OFFBOARD mode)
		rate = rospy.Rate(20.0)

		# Send a few waypoints before starting (just the current position)
		rospy.loginfo("Sending a few waypoints before starting (2 seconds)...")
		for i in range(40) :
			self.local_pos_pub.publish(self.pos)
			rate.sleep()

		# Wait for FCU connection
		rospy.loginfo("Waiting for FCU connection... (max 4 seconds)...")
		for i in range (80) :
			if self.state.connected :
				break
			rate.sleep()
		if not self.state.connected :
			rospy.logfatal("Error! Could not connect to FCU... Exiting now.")
			sys.exit()

		# Everything in order, let's fly
		rospy.loginfo("Everything in order. Let's fly.")
		last_request = rospy.get_rostime()
		prev_state = self.state
		flying_time = 0
		try:
			while not rospy.is_shutdown() and flying_time < FLY_TIME :
				now = rospy.get_rostime()

				if self.state.mode != "OFFBOARD" :
					if (now - last_request > rospy.Duration(2.)) :
						self.set_mode_client(base_mode=0, custom_mode="OFFBOARD")
						last_request = now
						rospy.logwarn("Setting offboard mode...")

				else:
					if not self.state.armed :
						if (now - last_request > rospy.Duration(2.)) :
					   		self.arming_client(True)
					   		last_request = now
					   		rospy.logwarn("Arming...")
					else :
					   	flying_time += 1
					   	# print("\r{}".format(flying_time)),
						sys.stdout.write('\r[FLYING #{}/{}] Objective: ({},{},{}). Current: ({},{},{}). Flight mode: {}.'.format(
							flying_time,
							FLY_TIME,
							self.next_pos.pose.position.x,
							self.next_pos.pose.position.y,
							self.next_pos.pose.position.z,
							self.pos.pose.position.x,
							self.pos.pose.position.y,
							self.pos.pose.position.z,
							self.flight_mode
						))

				# Check if state has changed
				if prev_state.armed != self.state.armed:
					rospy.loginfo("Vehicle armed: %r" % self.state.armed)

				if prev_state.mode != self.state.mode:
					rospy.loginfo("Current MAVROS state: %s" % self.state.mode)

				prev_state = self.state

				# Update objective position and publish it
				self.update_current_objective()
				self.local_pos_pub.publish(self.next_pos)
				rate.sleep()

		except KeyboardInterrupt :
			rospy.logerr('Keyboard Interrupt detected!')

		rospy.loginfo("Mission Completed. Landing...")
		rospy.wait_for_service('/mavros/cmd/land')

		#
		#	Try to land for 10 times
		#
		for i in range(10) :
			try:
				self.landing_client(altitude = 0, latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)
			except rospy.ServiceExceptio as e:
				print("service land call failed: {}. The vehicle cannot land".format(e))
			sleep(0.1)

	def update_state_cb(self, state) :
		'''
			Update MAVROS state
		'''
		self.state = state

	def update_current_pos_cb(self, pos) :
		'''
			Update position from mavros local pose.
			Some other node must be publishing there
		'''
		self.pos = pos

	def update_current_objective(self) :
		'''
			Updates objective position

			By default, the drone goes to home position + fixed height (stay mode)
			Then, depending on flight_mode :

				"stay"		: stay above home position at takeoff_height
				"hover" 	: keep current position (might drift...)
				"circle"	: do a circle of radius circle_radius
		'''

		self.next_pos = self.home_pos
		
		if self.flight_mode == "stay" :
			self.next_pos.pose.position.z = self.config_vars["takeoff_height"]
			self.next_pos.header.stamp = rospy.Time.now()

		if self.flight_mode == "hover" :
			self.next_pos = self.pos
			self.next_pos.header.stamp = rospy.Time.now()

		elif self.flight_mode == "circle" :
			theta = atan2(self.pos.pose.position.y, self.pos.pose.position.x)
			new_theta = (theta+0.5)%(2*pi)
			self.next_pos.pose.position.x = cos(new_theta)*self.config_vars["circle_radius"]
			self.next_pos.pose.position.y = sin(new_theta)*self.config_vars["circle_radius"]

			self.next_pos.pose.orientation.x = 0
			self.next_pos.pose.orientation.y = 0
			self.next_pos.pose.orientation.z = 0
			self.next_pos.pose.orientation.w = 1
			self.next_pos.pose.position.z = self.config_vars["takeoff_height"]
			self.next_pos.header.stamp = rospy.Time.now()

	def _pose_to_pose_dist(self, pose1, pose2) :
		'''
			Calculate the distance between two given poses
			Does not take into account orientation
		'''
		return norm(
				numpy_array([pose1.pose.position.x, pose1.pose.position.y, pose1.pose.position.z]) - 
				numpy_array([pose2.pose.position.x, pose2.pose.position.y, pose2.pose.position.z])
			)


if __name__ == '__main__':
	'''
		Run the node
	'''
	try:

		# Init mavros namespace
		mavros.set_namespace()

		# Create new control object
		control = Offboard_Control()
		control.flight_mode = "stay"

		# Run the mission control
		control.run()

	except rospy.ROSInterruptException:

		pass
