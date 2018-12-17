#!/usr/bin/env python

import rospy
import random
import RPi.GPIO as GPIO
import time
import threading

from car_control_v2 import Car_control
from std_msgs.msg import String

GPIO.setmode(GPIO.BCM)
GPIO.setup(22, GPIO.IN) # left collision
GPIO.setup(5, GPIO.IN)  # right collision

class Demo(object):
	def __init__(self):
		self.state = None
		self.last_state = 'pi'
		self.thread_l = threading.Thread(target = self.check_l_collision)
		self.thread_r = threading.Thread(target = self.check_r_collision)
		self.sub_state = rospy.Subscriber("/state", String, self.cb_state, queue_size = 5)
		self.first = True
		self.cc = Car_control()
		self.l_collision = None
		self.r_collision = None
	
	def check_l_collision(self):
		while 1:
			self.l_collision = GPIO.input(22)
			time.sleep(0.1)
			
	
	def check_r_collision(self):
		while 1:
			self.r_collision = GPIO.input(5)
			time.sleep(0.1)
	
	def avoid(self):
		if self.l_collision:
			self.cc.reverse(1.5)
			decision = random.randint(0, 1)
			if decision == 0:
				self.cc.turn('right', 1.5)
			else:
				self.cc.rotate_in_place('CCW', 1.5)
		if self.r_collision:
			self.cc.reverse(1.5)
			decision = random.randint(0, 1)
			if decision == 0:
				self.cc.turn('left', 1.5)
			else:
				self.cc.rotate_in_place('CW', 1.5)

	def start_thread(self):
		self.thread_l.start()
		self.thread_r.start()
	
	def cb_state(self, msg):
		self.state = msg.data
		if self.state != self.last_state:
			print "State changes from '{}' to '{}'".format(self.last_state, self.state)
		self.last_state = self.state

	def approach_ball(self):
		if self.first:
			print "Go straight"
			self.cc.go_straight(100, 3.5)
			print "Rotate in place"
			for i in range(4):
				self.cc.rotate_in_place('CW', 100, 0.2)
			print "Go straight again"
			self.cc.go_straight(110, 1.5)
			print "Rotate in place to find ball"
			while self.state == "pi":
				for i in range(4):
					self.cc.rotate_in_place('CW', 100, 0.2)
				if self.state == "pi":
					print "Not found in first rotate, keep rotating with different direction..."
					for i in range(9):
						self.cc.rotate_in_place('CCW', 100, 0.2)
			self.state = 'random'
			self.first = False
		while not rospy.is_shutdown():
			self.cc.go_straight(100, 2.0)
			self,collision()
	
	def shutdown(self):
		print "Shutdown in progress"
		self.thread_l._Thread__stop()
		self.thread_r._Thread__stop()
	
if __name__ == '__main__':
	rospy.init_node('demo_4_node')
	d = Demo()
	rospy.on_shutdown(d.shutdown)
	c = raw_input("Press any key to start: ")
	d.start_thread()
	d.approach_ball()
	rospy.spin()
