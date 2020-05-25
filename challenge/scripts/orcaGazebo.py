#! /usr/bin/python
# Copyright (c) 2013 Mak Nazecic-Andrlon 

"""

authors: Carlos Aunion Dominguez, Sergio Luzuriaga Rodriguez

"""


from __future__ import division

from pyorca import Agent, get_avoidance_velocity, orca, normalized, perp
from numpy import array, rint, linspace, pi, cos, sin, isnan, nan, arctan2
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from math import trunc
import tf

import rospy
import itertools
import random

class orca_ros:
	def __init__(self):

		# Robot radius
		self.radius = rospy.get_param('~radius', default=0.4)
		# Number of sections taken with the scanner
		self.sections = rospy.get_param('~sections', default=2)
		# Distance at which the robot acknowledges obstacles in its field of vision
		self.obstacle_distance = rospy.get_param('~obstacle_distance', default=1)
		# Maximum linear speed
		self.max_speed = rospy.get_param('~linear_max', default=0.5)
		# Maximum angular speed
		self.max_ang = rospy.get_param('~angular_max', default=0.3)
		# Parameter used to smooth the speed when accelerating
		self.smooth_param = rospy.get_param('~smooth_param', default=0.2)
		# Frecuency at which the robot operates (Hz)
		self.frecuency = rospy.get_param('~frecuency', default=10)
		# Variable used to store the previous speed sent to the robot
		self.pev_speed = (.0, .0)
		# Variable used to detect when the robot is likely stuck
		self.stuck_count = 0

		# Create a publisher which can "talk" to TurtleBot and tell it to move
		self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
		# Publishes obstacles markers
		self.marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=10)
		# Published if stuck
		self.planner_control = rospy.Publisher("planner/stuck", Bool, queue_size=10)

		# Subsriber that receives the robot's preferred speed from the control node
		rospy.Subscriber("/cmd_vel/tracker", Twist, self.callback_cmd_vel)
		# Subscriber that receives laser scan data from the gazebo node
		self.laser_scan = rospy.Subscriber("/scan", LaserScan, self.callback_scan)

		# Variable that activates when laser scan data is received for the first time
		self.laser_init = False
		# Array containing obstacles detected by the scan callback
		self.agents = ()

		# Used to transform points from one frame to another.
		self.listener = tf.TransformListener()

		


	""" Process control node's preferred speed into actual desired speed """
	def callback_cmd_vel(self, data):
		# If commanded speed isn't 0 and laser data has been received
		if data.linear.x != 0 and self.laser_init:
			# Transform angular speed received into linear speed used by orca
			linear_vel = self.angular2linear(data)
			# Create robot agent
			robot = Agent((0, 0), self.pev_speed, self.radius, self.max_speed, linear_vel)
			try:
				# Call orca method
				speed, line = orca(robot, self.agents, 1, 0.1)
				# Transform linear speed returned by orca into angular speed
				data = self.linear2angular(speed)
				# Smooth speed
				data = self.smooth_speed(data)
				# Raise exception if control node orders to keep advancing but orca orders to go back
				if(abs(linear_vel[0]) > 0.1 and data.linear.x < -0):
					# Increase stuck count variable
					self.stuck_count += 1
					print("Stuck detection: " + str(self.stuck_count))
					# When stuck count is higher than 15, the script will assume 
					# 	the robot is stuck and issue a replanning
					if self.stuck_count >= 7:
						# Reset stuck count varaible
						self.stuck_count = 0
						# Publish stuck topic to planner node
						self.planner_control.publish(Bool(True))
						raise Exception
			except:	#Exception raised if orca can't find a solution or the contidions above are met
				# Reduce speeds to ensure no collision
				data.linear.x *= 0.2
				data.angular.z *= 0.2
				
		# Store speed as previous speed
		self.pev_speed = self.angular2linear(data)
		# Publish speed to robot
		self.cmd_vel.publish(data)

	""" Avoid higher than maximum speeds and sudden accelerations """
	def smooth_speed(self, data):
		# Check if angular speed is above or below max limit
		if data.angular.z > self.max_ang:
			data.angular.z = self.max_ang
		elif data.angular.z < -self.max_ang:
			data.angular.z = -self.max_ang
		
		# Check if linear speed is above or below max limit
		if data.linear.x > self.max_speed:
			data.linar.x = self.max_speed
		elif data.linear.x < -self.max_speed:
			data.linear.x = -self.max_speed
		
		if data.linear.x < 0.002:
			# Decrease smooth parameter if robot isn't moving
			if self.smooth_param > 0.2:
				self.smooth_param -= 0.02
			# Apply smooth parameter
			data.linear.x *= self.smooth_param
		elif self.smooth_param < 1:
			# Apply smooth parameter
			data.linear.x *= self.smooth_param
			# Increase smooth parameter
			self.smooth_param += 0.05
		else:
			# Apply smooth parameter
			data.linear.x *= self.smooth_param
		return data

	""" Receive laser scan data and register obstacles """
	def callback_scan(self, data):
		# Get laser scan data
		self.laser_scan = data
		# Activate laser init variable
		self.laser_init = True		

		# Calculate skip step used to define sections in scan range
		skip = trunc(len(self.laser_scan.ranges)/self.sections)
		# Counter to keep record of current section
		cont = 1
		# List of obstacle agents
		agents = []
		# For each section:
		for i in range(1, len(self.laser_scan.ranges), skip):
			# If sections number wasn't surpassed
			if cont <= self.sections:
				# Initialize minimum distance and angle to NaN
				min_d = nan
				min_angle = nan
				# For each scan in the section
				for j in range(i, skip * cont):
					# Calculate the angle of the current scan
					angle = self.laser_scan.angle_min + j * self.laser_scan.angle_increment
					# Get distance of the current scan
					d = self.laser_scan.ranges[j]
					# Update minimum distance and angle if current distance is lower than 
					# 	the stored min_d and it's not a NaN, or the stored min_d is a NaN.
					if (not isnan(d) and d < min_d) or isnan(min_d):
						min_d = d
						min_angle = angle
				# Increase section counter
				cont += 1

				# If the min_d obtained is not a NaN and is lower than the defined obstacle distance
				if not isnan(min_d) and min_d < self.obstacle_distance:
					# Create obstacle as Point
					obstacle = Point(min_d * cos(min_angle), min_d * sin(min_angle), 0)
					# Create marker to visualize obstacles
					marker = Marker(
							type=Marker.SPHERE,
							id=i,
							lifetime=rospy.Duration(1.5),
							pose=Pose(obstacle, Quaternion(0, 0, 0, 1)),
							scale=Vector3(0.25, 0.25, 0.25),
							header=Header(frame_id='base_link'),
							color=ColorRGBA(0.0, 1.0, 0.0, 0.8))
					# Publish marker
					self.marker_publisher.publish(marker)
					# Create Agent at the obstacle's position with no speed and a low radius
					agent = Agent((obstacle.x, obstacle.y), (0, 0), 0.001, .0, (.0, .0))	
					# Add agent to list
					agents.append(agent)
		# Initialize agents variable with calculated obstacle agents
		self.agents = agents
	
	""" Transform angular speed into linear speed """
	def angular2linear(self, data):
		i_angular = data.angular.z / self.frecuency

		return (data.linear.x * cos(i_angular), data.linear.x * sin(i_angular))

	""" Transform linear speed into angular speed """
	def linear2angular(self, data):
		v_angular = arctan2(data[1], data[0]) * self.frecuency
		ret = Twist()
		ret.linear.x = data[0]
		ret.angular.z = v_angular

		return ret
    
	""" Stop orca node """
	def shutdown(self):
		rospy.loginfo("Stop orca")
		# Default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
		self.cmd_vel.publish(Twist())
		# Sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
		rospy.sleep(1)

if __name__ == '__main__':
	try:
		# Initialize node
		rospy.init_node('orca', anonymous=False)
		
		# Tell user how to stop TurtleBot
		rospy.loginfo("To stop TurtleBot CTRL + C")

		orca_r = orca_ros()

		#TurtleBot will stop if we don't keep telling it to move.  How often should we tell it to move? 10 HZ
		r = rospy.Rate(10);
		
		# What function to call when you ctrl + c    
		rospy.on_shutdown(orca_r.shutdown)
		count = 0
		while not rospy.is_shutdown():
			# The following code resets the stuck count variable 25 seconds after the variable was increased
			# 	to avoid replanning when the robot is capable of avoiding an obstacle without the need to replan
			if orca_r.stuck_count != 0:
				count += 1
				if(count > 250):
					orca_r.stuck_count = 0
					count = 0
				# Wait for 0.1 seconds (10 HZ)
			r.sleep()
	except:
		rospy.loginfo("Orca node terminated.")

	


