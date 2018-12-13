#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty, EmptyResponse
from std_msgs.msg import Int16

class Car_control(object):
	def __init__(self):
		self.pub_right_pwm = rospy.Publisher("right_pwm", Int16, queue_size = 1)
		self.pub_left_pwm  = rospy.Publisher("left_pwm", Int16, queue_size = 1)
		self.trim = rospy.get_param("~trim", -20)
		self.pwm_data = Int16()
		rospy.sleep(3.0)
	# Stop moving
	def stop_moving(self):
		self.pub_pwm(0, 0)
	# Go straight
	def go_straight(self, pwm, t)
		self.pub_pwm(pwm + self.trim, pwm - self.trim, t)
	# Rotate in place
	def rotate_in_place(self, direction, t)
		if direction == "CCW": # Counterclockwise
			self.pub_pwm(60, 0, t)
		elif direction == "CW": # Clockwise
			self.pub_pwm(0, 60, t)
	# Publish pwm data
	def pub_pwm(self, r_value, l_value, t):
		ts = rospy.Time.now().to_sec()
		while rospy.Time.now().to_sec() - ts <= t:
			pwm_data = r_value
			self.pub_right_pwm.publish(pwm_data)
			pwm_data = l_value
			self.pub_left_pwm.publish(pwm_data)
	def __del__(self):
		self.stop_moving()
		rospy.sleep(2.0)

if __name__ == "__main__":
	rospy.init_node("car_control_node")		
	cc = Car_control()
	cc.go_straight(100, 3)
	cc.stop_moving()
	rospy.sleep(1.0)
	cc.rotate_in_place('CW', 0.5)
	
