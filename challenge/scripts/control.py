#!/usr/bin/env python

"""

authors: Carlos Aunion Dominguez, Sergio Luzuriaga Rodriguez

"""

import sys
import math
import rospy
import tf
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool

class Turtlebot():
    def __init__(self):

		# Maximum linear speed allowed
		self.max_speed = rospy.get_param('~linear_max', default=0.5)
		# Maximum angular speed allowed
		self.max_speed_angular = rospy.get_param('~angular_max', default=0.3)
		# Distance the robot must be to a goal to consider it has reached it
		self.goal_tolerance = rospy.get_param('~goal_tolerance', default=0.2)
		# Threshold angle at which the robot rotates without moving
		self.angle_threshold = rospy.get_param('~ang_threshold', default=0.5)
		# Parameter to control robot linear speed
		self.speed_parameter = rospy.get_param('~speed_parameter', default=0.5)
		# Parameter to control robot angular speed
		self.angular_parameter = rospy.get_param('~angular_parameter', default=0.5)
		# Initialize goal lists
		self.goalx, self.goaly = [], []
		# Variable that controls if the next goal in path is the final goal
		self.is_goal = False
		# Variable that activates when a path is received
		self.path_received = False
		# Variable that activates when robot is stuck
		self.stuck = False

		self.replanning = False
			
		# Publishes preferred robot speeds to tracker topic, which orca node will use.
		self.cmd_vel = rospy.Publisher('/cmd_vel/tracker', Twist, queue_size=10)
		# self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)

		# Subscriber that receives the calculated path from the planner node
		self.path = rospy.Subscriber('/planner/path', Path, self.path_callback)

		# Used to transform points from one frame to another.
		self.listener = tf.TransformListener()

    def path_callback(self, path):
		""" Receive path from subscriber and store it in goal lists """
		# No new path will be generated until first goal of new path is reached
		# 	to avoid consecutive replannings
		if not self.replanning:
			# Activate replanning flag
			self.replanning = True
			self.goalx, self.goaly = [], []
			# Activate stuck variable
			self.stuck = True
			# Add path points to lists
			for pose in path.poses:
				self.goalx.append(pose.pose.position.x)
				self.goaly.append(pose.pose.position.y)
			# Activate path received variable
			self.path_received = True

    def command(self,gx, gy):
		""" Calculate preferred speed to reach current goal """
		# Initialize PointStamped object on goal coordinates in '/map' frame
		goal = PointStamped()
		goal.header.frame_id = "/map"
		goal.header.stamp = rospy.Time()
		goal.point.x = gx
		goal.point.y = gy
		goal.point.z = 0.0

		# Create PointStamped object to store goal coordinates in '/base_link' frame
		base_goal = PointStamped()
		
		try: 	# Transform goal point to '/base_link' frame
			base_goal = self.listener.transformPoint('base_link', goal)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			rospy.loginfo("Problem TF")
			return

		# Obtain alpha
		alpha = math.atan2(base_goal.point.y, base_goal.point.x)
		# Calculate distance to goal
		dist = math.sqrt(math.pow(base_goal.point.x,2)+math.pow(base_goal.point.y,2))
		
		# Control distance parameter so it's never lower than 0.5
		if dist < 0.5:
			dist_param = 0.5
		else:
			dist_param = dist
		
		# If alpha is higher than the defined angle threshold
		if math.fabs(alpha) > self.angle_threshold:
			# Set threshold to more restricted value while rotating
			self.angle_threshold = 0.3
			# Calculate required angular speed
			angular = self.angular_parameter * alpha
			# Stop linear movement
			linear = 0.0
		# If the distance to goal is higher than the defined goal tolerance
		elif dist > self.goal_tolerance:
			# Restore angle threshold value
			self.angle_threshold = rospy.get_param('~ang_threshold', default=0.5)
			# Calculate required angular speed
			angular = self.angular_parameter * alpha
			# Calculate required linear speed
			linear = self.speed_parameter * dist_param * (1 - abs(angular))
		else:
			# If next goal isn't the final goal keep advancing at reduced rate
			if(not self.is_goal):
				# Calculate required angular speed and reduce it by 50%
				angular = self.angular_parameter * alpha * 0.5
				# Calculate required linear speed and reduce it by 50%
				linear = self.speed_parameter * dist_param * (1 - abs(angular)) * 0.5
			# Stop
			else:
				angular = 0.0
				linear = 0.0
			# Return true if goal is reached
			return True
		# Call to publish method with preferred speeds
		self.publish(linear,angular)

    def publish(self, lin_vel, ang_vel):
		# Create Twist to store speeds
		move_cmd = Twist()
		# Assure published linear speed isn't higher than defined max
		if lin_vel > self.max_speed:
			lin_vel = self.max_speed
		# Assure published angular speed isn't higher than defined max
		if ang_vel > self.max_speed_angular:
			ang_vel = self.max_speed	

		move_cmd.linear.x = lin_vel
		move_cmd.angular.z = ang_vel
		# Publish preferred speed to orca node
		self.cmd_vel.publish(move_cmd)

    def shutdown(self):
        # Stop turtlebot
        rospy.loginfo("Stop TurtleBot")
 
if __name__ == '__main__':
    try:
		rospy.init_node('robotcontrol', anonymous=False)
		rospy.loginfo("To stop TurtleBot CTRL + C")
		robot = Turtlebot()
	 	# What function to call when you ctrl + c    
		rospy.on_shutdown(robot.shutdown)

		# Publish rate: 10 HZ
		r = rospy.Rate(10);
		
		while not rospy.is_shutdown():
			# If a path has been received
			if robot.path_received:
				# Reset goal reached varaible
				goal_reached = False
				# While final goal hasn't been reached
				while not goal_reached:
					# Get path from variable
					x_path, y_path = robot.goalx, robot.goaly
					
					rospy.loginfo("Following path...")
					# Ignore first point in path
					goalx = x_path.pop(0)
					goaly = y_path.pop(0)
					# Get first goal in path
					goalx = x_path.pop(0)
					goaly = y_path.pop(0)
					rospy.loginfo("First goal set: (" + str(goalx) + ", "+str(goaly)+")")
					# Reset stuck variable
					robot.stuck = False
					# Reset is goal variable
					robot.is_goal = False	
					# Reset goal reached variable
					goal_reached = False
					# Restore goal tolerance value
					robot.goal_tolerance = rospy.get_param('~goal_tolerance', default=0.2)
					# Restore angle threshold value
					robot.angle_threshold = rospy.get_param('~angle_threshold', default=0.2)
					# While there are path points remaining
					while len(x_path) > 0:
						# If the robot is stuck, obtain new path
						if robot.stuck:
							# Get new path from variable
							x_path = robot.goalx
							y_path = robot.goaly
							# Ignore first point
							goalx = x_path.pop(0)
							goaly = y_path.pop(0)
							# Get next point
							goalx = x_path.pop(0)
							goaly = y_path.pop(0)
							# Reset stuck variable
							robot.stuck = False
						
						# Check if the robot has reached the current goal and command speed
						if robot.command(goalx,goaly):
							rospy.loginfo("Next goal set: (" + str(goalx) + ", "+str(goaly)+")")
							# When first goal is reached, replanning is allowed again
							robot.replanning = False
							# Get next goal in path
							goalx = x_path.pop(0)
							goaly = y_path.pop(0)
						# Wait for 0.1 seconds (10 HZ) and try again
						r.sleep()
					# When current goal is the last path goal
					rospy.loginfo("Last goal set: (" + str(goalx) + ", "+str(goaly)+")")
					# Update is goal variable
					robot.is_goal = True	
					# Restrict goal tolerance and angle threshold variable to get closer to the final goal
					robot.goal_tolerance = 0.05
					robot.angle_threshold = 0.05
					
					# While robot hasn't reached last goal
					while not robot.command(goalx, goaly):
						# If stuck, break while
						if robot.stuck:
							print("Stuck before goal")
							# Reset stuck variable
							robot.stuck = False
							break
						# wait for 0.1 seconds (10 HZ) and try again
						r.sleep()
					# If robot has reached the final goal
					if robot.command(goalx, goaly):
						rospy.loginfo("Goal reached: (" + str(goalx) + ", "+str(goaly)+")")
						# Activate goal reached variable
						goal_reached = True
						# Reset path received varaible
						robot.path_received = False
			# wait for 0.1 seconds (10 HZ) and publish again
			r.sleep()

    except:
        rospy.loginfo("Robot control node terminated.")