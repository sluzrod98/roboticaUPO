#!/usr/bin/env python

""" A ROS planning node that subscribes to a costmap
  and generates a path by using the AStar algorithm"""

import math
import sys

import rospy
import tf
from geometry_msgs.msg import Point, PoseStamped, PointStamped
from nav_msgs.msg import OccupancyGrid, Path
from visualization_msgs.msg import Marker
from std_msgs.msg import Bool
from AStar import AStar
from time import sleep


class Planner:
	def __init__(self):
		rospy.loginfo("In the planner constructor")
		# Try to get current robot position as init point
		try:
			self.current_pos = PointStamped()
			self.current_pos = self.get_current_position()
			self.initx = self.current_pos.point.x
			self.inity = self.current_pos.point.y
		# If getting the current position fails, get init point from parameter server
		except:
			self.initx = rospy.get_param('~init/x')
			self.inity = rospy.get_param('~init/y')
		# Get goal point from parameter server
		if rospy.has_param('~goal'):
			self.goalx = rospy.get_param('~goal/x')
			self.goaly = rospy.get_param('~goal/y')
		
		print('Init (%f, %f). Goal (%f,%f): '%(self.initx, self.inity, self.goalx,self.goaly))

		# Threshold at which a costmap value is considered an obstacle
		self.obstacle_threshold = rospy.get_param('~obstacle_threshold', default=95)

		# Used to transform points from one frame to another.
		self.listener = tf.TransformListener()

		# This flag would be raised in the map callback when map data is received
		self.init = False 

		# Publishes path data to control node
		self.path_pub = rospy.Publisher('~path', Path, queue_size=10)
		# Publishes path markers to visualize in Rviz
		self.marker_pub = rospy.Publisher('~path_marker', Marker, queue_size=10)

		# Receives costmap data from the map server
		rospy.Subscriber('costmap_2d/costmap/costmap', OccupancyGrid, self.map_callback)
		rospy.Subscriber('goal_pose', OccupancyGrid, self.map_callback)
		# Receives flag when orca node thinks robot is stuck
		rospy.Subscriber("planner/stuck", Bool, self.stuck_callback)


	def stuck_callback(self, stuck):
		""" Initiate new path planning at current position if robot is stuck """
		# Create new init point
		map_init = PointStamped();
		# Get current position
		map_init = self.get_current_position()
		# Update init variables
		self.initx = map_init.point.x
		self.inity = map_init.point.y

		# Activate init flag
		self.init = not stuck.data

	def get_current_position(self):
		""" Returns current robot position on '/map' frame """
		init = PointStamped();
		#Create PointStamped at 0,0 coordinates in '/base_link' frame
		map_init = PointStamped();
		init.header.frame_id = "/base_link";
		init.header.stamp = rospy.Time();
		init.point.x = 0.0;
		init.point.y = 0.0;
		init.point.z = 0.0;

		# Transform point to '/map' frame
		try:
			map_init = self.listener.transformPoint('map', init)
			return map_init
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			rospy.loginfo("Problem TF")
			return

	def map_callback(self, map):
		""" Get map info and calculate path """
		# Update map data
		self.map = map
		# If init flag wasn't rised
		if not self.init:
			# Raise init flag
			self.init = True
			# Calculate path using init and goal points
			self.path = self.calculate_path(self.initx, self.inity, self.goalx, self.goaly)
			# Call to publish path method
			self.publish_path()
			# Call to publish path marker method
			self.publish_path_marker()

	def calculate_path (self, ix, iy, gx, gy):
		""" Calculate best path using an A* algorithm """
		self.astar = AStar(self.map, self.obstacle_threshold)
		return self.astar.planning(ix, iy, gx, gy)

	def publish_path(self):
		""" Create and publish path to control node """
		# Initialize path message
		self.path_msg = Path()
		poses = []
		# Add points to path
		for i in range(len(self.path[0])):
			pose = PoseStamped()
			pose.header.stamp = rospy.get_rostime()
			pose.header.frame_id = "/map"
			pose.pose.position.x = self.path[0][i]
			pose.pose.position.y = self.path[1][i]
			poses.append(pose)
		# Define path parameters
		self.path_msg.header.stamp = rospy.get_rostime()
		self.path_msg.header.frame_id = "/map"
		self.path_msg.poses = poses
		# Publish path to control node
		self.path_pub.publish(self.path_msg)
  
	def publish_path_marker(self):
		""" Create and publish path markers to visualize in Rviz """
		# Create marker
		path_marker = Marker()
		# Get x and y arrays from path
		x, y = self.path
		print ("Publishing Markers...")
		# Initialize marker parameters
		path_marker.id = 0
		path_marker.header.frame_id = "/map"
		path_marker.header.stamp = rospy.get_rostime()
		path_marker.type = path_marker.LINE_STRIP
		path_marker.action = path_marker.ADD
		path_marker.ns = "path"
		path_marker.scale.x = 0.1
		path_marker.color.g = 1
		path_marker.color.a = 1
		# Add points to marker
		for i in range(len(self.path[0])):
			p = Point(x=x[i], y=y[i], z=0)
			path_marker.points.append(p)
		# Publish marker to topic
		self.marker_pub.publish(path_marker)
 
if __name__ == '__main__':
    try:
		# sleep 3 sec to let obstacles load properly on the map
		sleep(3)

		# initiliaze
		rospy.init_node('planning', anonymous=False)

		# tell user how to stop TurtleBot
		rospy.loginfo("Starting planning node. Waiting for valid map")

		planner = Planner()     

		# The planning node will wait for new goals and the map at this rate
		r = rospy.Rate(10)

		# as long as you haven't ctrl + c keeping doing...
		while not rospy.is_shutdown():
			r.sleep()

    except:
        rospy.loginfo("Planning node terminated.")