#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16

class Car_control(object):
	def __init__(self):
		self.pub_right_pwm = rospy.Publisher("right_pwm", Int16, queue_size = 10)
		self.pub_left_pwm  = rospy.Publisher("left_pwm", Int16, queue_size = 10)
		self.trim = rospy.get_param("~trim", 0)
		print "trim: ", self.trim
		self.pwm = 100
		self.pwm_data = Int16()
		rospy.sleep(3.0) # sleep 3.0 second to ensure topics connection
	def stop_moving(self):
		# stop moving
		self.pub_pwm(0, 0)
	def go_straight(self):
		# go straight
		self.pub_pwm(self.pwm + self.trim, self.pwm - self.trim)
	def left_collision_recovery(self):
		# since left collision, we have to reverse and turn right
		# stop first
		self.stop_moving()
		rospy.sleep(0.5)
		self.pub_pwm(-self.pwm - self.trim, -self.pwm + self.trim)
		rospy.sleep(2.0) # reverse 2.0 seconds
		self.pub_pwm(self.pwm/2, self.pwm)
	def right_collision_recovery(self):
		# since right collison, we have to reverse and turn left
		# stop first
		self.stop_moving()
		rospy.sleep(0.5)
		self.pub_pwm(-self.pwm - self.trim, -self.pwm + self.trim)
		rospy.sleep(2.0) # reverse 2.0 seconds
		self.pub_pwm(self.pwm, self.pwm/2)
	def pub_pwm(self, r_value, l_value):
		pwm = Int16(r_value)
		print "right pwm: ", pwm.data
		self.pub_right_pwm.publish(pwm)
		pwm.data = l_value
		print "left pwm: ", pwm.data
		self.pub_left_pwm.publish(pwm)
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
