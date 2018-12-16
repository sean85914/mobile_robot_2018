#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty, EmptyResponse
from std_msgs.msg import Int16

class Car_control(object):
	def __init__(self):
		self.pub_right_pwm = rospy.Publisher("right_pwm", Int16, queue_size = 5)
		self.pub_left_pwm  = rospy.Publisher("left_pwm", Int16, queue_size = 5)
		self.trim = rospy.get_param("/demo_4_node/trim", -20)
		print "Trim is now: ", self.trim
		self.pwm_data = Int16()
		rospy.sleep(3.0)
	# Stop moving
	def stop_moving(self, t):
		self.pub_pwm(0, 0, t)
	# Go straight
	def go_straight(self, pwm, t):
		self.pub_pwm(pwm + self.trim, pwm - self.trim, t)
		self.stop_moving(0.5)
	# Rotate in place
	def rotate_in_place(self, direction, pwm, t):
		if direction == "CCW": # Counterclockwise
			self.pub_pwm(pwm, -pwm, t)
			self.stop_moving(0.5)
		elif direction == "CW": # Clockwise
			self.pub_pwm(-pwm, pwm, t)
			self.stop_moving(0.5)
	# Reverse
	def reverse(self, t):
		self.pub_pwm(-100 - self.trim, -100 + self.trim, t)
		self.stop_moving(0.5)
	# Turn
	def turn(self, direction, t):
		if direction == "Left" or direction == "left":
			self.pub_pwm(100, 80, t)
			self.stop_moving(0.5)
		elif direction == "Right" or direction == "right":
			self.pub_pwm(80, 110, t)
			self.stop_moving(0.5)
	# Publish pwm data
	def pub_pwm(self, r_value, l_value, t):
		ts = rospy.Time.now().to_sec()
		while rospy.Time.now().to_sec() - ts <= t:
			pwm_data = r_value
			self.pub_right_pwm.publish(pwm_data)
			pwm_data = l_value
			self.pub_left_pwm.publish(pwm_data)
			rospy.sleep(0.1)
	# Delete class function
	def __del__(self):
		print "Shutdown..."
		self.stop_moving(2.0)
	# ROS shutdown function
	def shutdown(self):
		pass

if __name__ == "__main__":
	rospy.init_node("car_control_node")		
	cc = Car_control()
	rospy.on_shutdown(cc.shutdown)
	#print "go straight"
	#cc.go_straight(80, 2)
	#print "stop"
	#cc.stop_moving(1.0)
	#print "rotate in place"
	#cc.rotate_in_place('CW', 1)
	#cc.stop_moving(1.0)
	#cc.rotate_in_place('CCW', 1)
	#cc.stop_moving(1.0)
	print "reverse"
	cc.reverse(3)
	#print "turn left"
	#cc.turn("Left", 1)
	#print "stop"
	#cc.stop_moving(1)
	#print "turn right"
	#cc.turn("Right", 1)
	#cc.stop_moving(1.0)
	rospy.signal_shutdown("End")	
