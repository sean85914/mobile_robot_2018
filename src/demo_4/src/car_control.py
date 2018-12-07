#!/usr/bin/env python

import random
import rospy
from std_msgs.msg import Int16

class Car_control(object):
	def __init__(self):
		self.pub_right_pwm = rospy.Publisher("right_pwm", Int16, queue_size = 10)
		self.pub_left_pwm  = rospy.Publisher("left_pwm", Int16, queue_size = 10)
		self.trim = rospy.get_param("~trim", -20)
		#print "trim: ", self.trim
		self.pwm = 120
		self.pwm_data = Int16()
		rospy.sleep(3.0) # sleep 3.0 second to ensure topics connection
	# Stop the robot
	def stop_moving(self):
		# stop moving
		self.pub_pwm(0, 0)
	# Go straight
	def go_straight(self):
		# go straight
		self.pub_pwm(self.pwm + self.trim - 10, self.pwm - self.trim)
	# Go straight but faster than above one
	def go_faster(self):
		self.pub_pwm(self.pwm + self.trim + 30 - 10, self.pwm - self.trim + 30)
	# When left touch sensor collision, recover the robot
	# Process:
	#   1. Stop moving first
	#   2. Go reverse with 0.5 seconds
	#   3. Spin the robot or turn right, decided by random number
	def left_collision_recovery(self):
		# since left collision, we have to reverse and turn right
		# stop first
		self.stop_moving()
		rospy.sleep(0.5)
		self.pub_pwm(-self.pwm - self.trim, -self.pwm + self.trim)
		rospy.sleep(0.5) # reverse 0.5 seconds
		rand = random.randint(0, 3)
		if rand != 0:
			rand_ = random.randint(0, 1)
			if rand_ == 0:
				self.rotate_heading('CCW')
			else:
				self.rotate_heading('CW')
			return 'spin'
		else:
			self.pub_pwm(self.pwm, self.pwm + 30) # turn right
			return 'right'
	# Robot hit the left touch sensor while it carries the ball,
	# we cannot reverse with too much distance so this is different 
	# from the above one
	def left_collision_recovery_with_ball(self):
		self.stop_moving()
		rospy.sleep(0.5)
		ts = rospy.Time.now().to_sec()
		while rospy.Time.now().to_sec() - ts < 0.2:
			self.pub_pwm(-90, -90)
		self.rotate_heading('CW')
	# When right touch sensor collision, recover the robot
	# Process:
	#   1. Stop moving the robot
	#   2. Go reverse with 0.5 seconds
	#   3. Spin the robot or turn left, decided by random number
	def right_collision_recovery(self):
		# since right collison, we have to reverse and turn left
		# stop first
		self.stop_moving()
		rospy.sleep(0.5)
		self.pub_pwm(-self.pwm - self.trim, -self.pwm + self.trim)
		rospy.sleep(0.5) # reverse 0.5 seconds
		rand = random.randint(0, 3)
		if rand != 0:
			rand_ = random.randint(0, 1)
			if rand_ == 0:
				self.rotate_heading('CCW')
			else:
				self.rotate_heading('CW')
			return 'spin'
		else:
			self.pub_pwm(self.pwm + 30, self.pwm) # turn left
			return 'left'
	def right_collision_recovery_with_ball(self):
		self.stop_moving()
		rospy.sleep(0.5)
		ts = rospy.Time.now().to_sec()
		while rospy.Time.now().to_sec() - ts < 0.2:
			self.pub_pwm(-90, -90)
		self.rotate_heading('CCW')
	# Rotate the robot
	# Parameter:
	#   rotate_type: how to rotate the robot, default to "CCW"
	def rotate_heading(self, rotate_type = "CCW"):
		if rotate_type == "CCW": # Counterclockwise, r > 0, l < 0
			ts = rospy.Time.now().to_sec()
			while rospy.Time.now().to_sec() - ts < 0.1:
				self.pub_pwm(self.pwm/1.5, -self.pwm/1.5)
			self.stop_moving()
			
		elif rotate_type == "CW": # Clockwise, r < 0, l > 0
			ts = rospy.Time.now().to_sec()
			while rospy.Time.now().to_sec() - ts < 0.1:
				self.pub_pwm(-self.pwm/1.5, self.pwm/1.5)
			self.stop_moving()
	# Go toward the door
	def go_toward_door(self):
		ts = rospy.Time.now().to_sec()
		while rospy.Time.now().to_sec() - ts < 0.3:
			self.pub_pwm(self.pwm, self.pwm)
	# Publish pwm value of two wheels to Arduino
	# Parameters:
	#  r_value: right wheel pwm value
	#  l_value: left  wheel pwm value
	def pub_pwm(self, r_value, l_value):
		pwm = Int16(r_value)
		print "right pwm: ", pwm.data
		self.pub_right_pwm.publish(pwm)
		pwm.data = l_value
		print "left pwm: ", pwm.data
		self.pub_left_pwm.publish(pwm)
                rospy.sleep(1.0)
	def __del__(self):
		rospy.loginfo("stop the vehicle")
		rospy.sleep(1.0)
		self.stop_moving()
		rospy.sleep(2.0)
	def shutdown(self):
		pass

if __name__ == "__main__":
	rospy.init_node("car_control")
	car_control = Car_control()
	rospy.loginfo("Go straight 3.0 seconds...")
	car_control.go_straight()
	rospy.sleep(3.0)
	rospy.loginfo("Left collision recovery, turn right 2.0 seconds...")
	car_control.left_collision_recovery()
	rospy.sleep(2.0)
	rospy.loginfo("Right collision recovery, turn left 2.0 seconds...")
	car_control.right_collision_recovery()
	rospy.sleep(2.0)
	rospy.loginfo("Stop the vehicle...")
	car_control.stop_moving()
	rospy.loginfo("Shutdown the node")
	rospy.on_shutdown(car_control.shutdown)
