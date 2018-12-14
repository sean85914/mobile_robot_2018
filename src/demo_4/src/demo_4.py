#!/usr/bin/env python

import rospy
import random
import time
import RPi.GPIO as GPIO
import threading

from car_control_v2 import Car_control

GPIO.setmode(GPIO.BCM)
GPIO.setup(27, GPIO.IN) # photo collision
GPIO.setup(22, GPIO.IN) # left collision
GPIO.setup(5, GPIO.IN)  # right collision
GPIO.setup(21, GPIO.IN) # photo sensor
GPIO.setup(20, GPIO.IN) # IR sensor

global cc
cc = Car_control()

def in_range(data, up, low):
	if data > low and data < up:
		return True
	else:
		return False

class Demo(object):
	def __init__(self):
		self.first_find_target = False
		self.start_tag = rospy.get_param("~start_tag", "A") # A / C
		self.door_frequency = rospy.get_param("~door_frequency", 600) # 600 / 1500
		print "Start from {}, go to door {}".format(self.start_tag, str(self.door_frequency))
		self.bound = [0.23, 0.16, 0.33, 0.26] # [0:2] -> 1500, [2:4] -> 600
		self.thread_1 = threading.Thread(target = self.check_l_collision)
		self.thread_2 = threading.Thread(target = self.check_r_collision)
		self.thread_3 = threading.Thread(target = self.check_p_collision)
		self.thread_4 = threading.Thread(target = self.check_find_light)
		self.thread_5 = threading.Thread(target = self.check_ir)
		self.thread_1.start()
		self.thread_2.start()
		self.thread_3.start()
		self.thread_4.start()
		self.thread_5.start()

	def check_l_collision(self):
		while 1:
			if GPIO.input(22):
				print "Left collision"
				cc.stop_moving(0.5)
				cc.reverse(2)
				i = random.randint(0, 1)
				if i == 0:
					cc.rotate_in_place('CW', 80, 1)
				else:
					cc.turn('Right', 2)

	def check_r_collision(self):
		while 1:
			if GPIO.input(5):
				print "Right collision"
				cc.stop_moving(0.5)
				cc.reverse(2)
				i = random.randint(0, 1)
				if i == 0:
					cc.rotate_in_place('CCW', 80, 1)
				else:
					cc.turn('Left', 2)

	def check_p_collision(self):
		while 1:
			if GPIO.input(27) and not self.first_find_target:
				print "Photo collision"
				cc.stop_moving(0.5)
				self.first_find_target = True
			if self.first_find_target:
				pass

	def check_find_light(self):
		while 1:
			if not GPIO.input(21) and not self.first_find_target:
				print "Find light"
				cc.stop_moving(0.5)
				cc.go_straight(120, 2)
			else:
				pass # We have found light, so don't sprint
	def check_ir(self):
		while 1:
			if not self.first_find_target:
				pass
			else:
				print "find door stage"
				cc.rotate_in_place('CCW', 110, 0.2)
				ts = time.time()
				detect_list = []
				while time.time() - ts <= 0.2:
					detect_list.append(GPIO.input(20))
				list_len = len(detect_list)
				data_0_len = 0
				for i in range(list_len):
					if detect_list[i] == 0:
						data_0_len += 1
				ratio = data_0_len / float(list_len)
				print "ratio: ", ratio
				if self.door_frequency == 1500:
					door = 0
				else:
					door = 2
				if in_range(ratio, self.bound[door], self.bound[door+1]):
					cc.go_straight(120, 2)
				else:
					if self.start_tag == 'A':
						ts = time.time()
						cc.rotate_in_place('CCW', 90, 0.5)
					else:
						ts = time.time()
						cc.rotate_in_place('CW', 90, 0.5)
	def get_tag(self):
		return self.start_tag

	def get_state(self):
		return self.first_find_target

if __name__ == "__main__":
	rospy.init_node("demo_4_node")
	d = Demo()
	print "Go straight"
	cc.go_straight(100, 4)
	print "Turn"
	if d.get_tag() == 'A':
		# Turn right
		cc.turn('Right', 2)
	elif d.get_tag() == 'C':
		# Turn left
		cc.turn('Left', 2)
	ts = time.time()
	print "Start rotate in place"
	while time.time() - ts <= 10:
		if d.get_tag() == 'A':
			cc.rotate_in_place('CW', 90, 0.3)
		else:
			cc.rotate_in_place('CCW', 90,0.3)
	if d.get_state() != "True":
		print "Not reach, start arbitrary traversing"
		while not rospy.is_shutdown():
			cc.go_straight(100, 2)
			cc.rotate_in_place('CW', 90, 0.5)

