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
		self.state = None # Now state = enum{'pi', 'random', 'move toward ball', 'find door', 'move toward door'}
		self.last_state = 'pi' # Last state, for diagnosing information
		self.thread_l = threading.Thread(target = self.check_l_collision) # Left touch sensor thread
		self.thread_r = threading.Thread(target = self.check_r_collision) # Right touch sensor thread
		self.sub_state = rospy.Subscriber("/state", String, self.cb_state, queue_size = 5) # Subscribe state for arduino
		self.first = True # First execution
		self.cc = Car_control() # Car control class
		self.l_collision = None # Left touch sensor state
		self.r_collision = None # Right touch sensor state
		self.ts = None # Start time, initialize when start thread
		self.thread_has_started = False # Whether if threads have started
	# Left touch sensor thread callback
	def check_l_collision(self):
		while 1:
			self.l_collision = GPIO.input(22)
			time.sleep(0.1)
	# Right touch sensor thread callback
	def check_r_collision(self):
		while 1:
			self.r_collision = GPIO.input(5)
			time.sleep(0.1)
	# When 'random' and there is a collision, avoid the obstacle
	# Reverse -> Turn or Rotate in place
	def avoid(self):
		# Left collision, turn right or rotate CCW
		if self.l_collision:
			self.cc.reverse(1.5)
			decision = random.randint(0, 1)
			if decision == 0:
				self.cc.turn('right', 1.5)
			else:
				self.cc.rotate_in_place('CCW', 100, 0.5)
		# Right collision, turn left or rotate CW
		if self.r_collision:
			self.cc.reverse(1.5)
			decision = random.randint(0, 1)
			if decision == 0:
				self.cc.turn('left', 1.5)
			else:
				self.cc.rotate_in_place('CW', 100, 0.5)
	# After user press key, start the thread
	def start_thread(self):
		self.thread_l.start()
		self.thread_r.start()
		self.ts = time.time()
		self.thread_has_started = True
	# /state callback function, update state and print information if state change
	def cb_state(self, msg):
		self.state = msg.data
		if self.state != self.last_state and self.thread_has_started:
			print "[Time: {}] State changes from '{}' to '{}'".format(time.time() - self.ts, self.last_state, self.state)
		self.last_state = self.state
	# Mainly loop in the class
	# Target: approach ball
	# Method: if first execution, then just go straight, turn, go straight again then the robot might see the light, 
	#         if, unfortunately, not see the light, then start to rotate in place CCW and CW respectively with go straight
	#         twice. After that, change state from 'pi' to 'random'. 
	#         When state is 'random', then the robot will just go straight for few seconds, and rotate CCW or CW randomly
	def approach_ball(self):
		if self.first:
			print "Go straight"
			self.cc.go_straight(100, 3.5)
			print "Rotate in place"
			for i in range(4):
				self.cc.rotate_in_place('CW', 100, 0.2)
			print "Go straight again"
			self.cc.go_straight(110, 2.0)
			print "Rotate in place to find ball"
			while self.state == "pi":
				for i in range(2):
					for j in range(4):
						self.cc.rotate_in_place('CW', 100, 0.2)
					if self.state == "pi":
						print "Not found in first rotate, keep rotating with different direction..."
						for j in range(4):
							self.cc.rotate_in_place('CCW', 100, 0.2)
					self.cc.go_straight(100, 1.0)
			self.state = 'random'
			self.last_state = 'pi'
			print "[Time: {}] State changes from 'pi' to 'random'".format(time.time() - self.ts)
			self.first = False
		while not rospy.is_shutdown():
			self.cc.go_straight(100, 2.0)
			self.avoid()
			direct_decide = random.randint(0, 1)
			if direct_decide == 0:
				direct = 'CCW'
			else:
				direct = 'CW'
			self.cc.rotate_in_place(direct, 100, 0.5)
			print "[Time: {}]".format(time.time() - self.ts)
	# When shutdown is called, stop the threads
	def shutdown(self):
		print "Shutdown in progress"
		self.thread_l._Thread__stop()
		self.thread_r._Thread__stop()
	
if __name__ == '__main__':
	rospy.init_node('demo_4_node')
	d = Demo()
	rospy.on_shutdown(d.shutdown)
	# Block until user input
	c = raw_input("Press any key to start: ")
	d.start_thread()
	d.approach_ball()
	rospy.spin()
