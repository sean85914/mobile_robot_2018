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

def in_range(data, up, low):
	if data > low and data < up:
		return True
	else:
		return False

class Demo(object):
	def __init__(self):
		self.find_target = False # if find target
		self.start_tag = rospy.get_param("~start_tag", "A") # A / C
		self.door_frequency = rospy.get_param("~door_frequency", 600) # 600 / 1500
		print "Start from {}, go to door {}".format(self.start_tag, str(self.door_frequency))
		self.bound = [0.23, 0.16, 0.33, 0.26] # [0:2] -> 1500, [2:4] -> 600
		self.thread_l = threading.Thread(target = self.check_l_collision)
		self.thread_r = threading.Thread(target = self.check_r_collision)
		self.thread_p = threading.Thread(target = self.check_p_collision)
		self.thread_ph = threading.Thread(target = self.check_find_light)
		self.thread_ir = threading.Thread(target = self.check_ir)
		self.first = True
		self.l_collision = None
		self.r_collision = None
		self.ph_collision = None
		self.photo_state = None
		self.near_door = None
		self.flag = None
		self.cc = Car_control()
		self.no_collide = 0

	def check_l_collision(self):
		while 1:
			self.l_collision = GPIO.input(22)
			time.sleep(0.1)

	def check_r_collision(self):
		while 1:
			self.r_collision = GPIO.input(5)
			time.sleep(0.1)

	def check_p_collision(self):
		while 1:
			self.find_target = GPIO.input(27)
			if self.find_target:
				print "Find target"
				self.flag = True
			else:
				self.flag = False
			time.sleep(0.1)

	def check_find_light(self):
		while 1:
			if not self.find_target:
				if GPIO.input(21):
					self.photo_state = False
				else:
					self.photo_state = True
					#print "Find light"
			time.sleep(0.1)

	def check_ir(self):
		while 1:
			if self.find_target:
				data_arr = []
				ts = time.time()
				while time.time() - ts <= 0.2:
					data_arr.append(GPIO.input(20))
				data_len = len(data_arr)
				data_0_len = 0
				for i in range(data_len):
					if data_arr[i] == 0:
						data_0_len += 1
				ratio = data_0_len / float(data_len)
				if self.door_frequency == 1500:
					door_idx = 0
				else:
					door_idx = 1
				if in_range(ratio, self.bound[door_idx], self.bound[door_idx+1]):
					self.near_door = True
					print "Find door ", self.door_frequency
				else:
					self.near_door = False

	def start_thread(self):
		self.thread_l.start()
		self.thread_r.start()
		self.thread_p.start()
		self.thread_ph.start()
		self.thread_ir.start()
	
	def approach_ball(self):
		print "Go straight"
		self.cc.go_straight(100, 3.5)
		print "Rotate in place"
		if self.start_tag == 'A':
			direct = 'CW'
		else:
			direct = 'CCW'
		for i in range(3):
			self.cc.rotate_in_place(direct, 90, 0.3)
		print "Go straight again"
		self.cc.go_straight(110, 1.5)
		print "Rotate in place to find ball"
		ts = time.time()
		find_tp = None
		collide = None
		while time.time() - ts <= 6:
			self.cc.rotate_in_place(direct, 90, 0.4)
			if self.photo_state:
				print "Break since find light"
				find_tp = True
				break
		if not find_tp:
			print "Not found, go forward a little bit"
			self.cc.go_straight(100, 1)
			if self.start_tag == "A":
				direct = 'CCW'
			else:
				direct = 'CW'
			print "Rotate in place"
			ts = time.time()
			while time.time() - ts <= 6:
				self.cc.rotate_in_place(direct, 90, 0.4)
				if self.photo_state:
					print "Break since find light"
					find_tp = True
					break
		if self.photo_state or find_tp or self.flag:
			if not self.find_target:
				print "Find light, move toward it"
				self.cc.stop_moving(0.3)
				self.cc.go_straight(120, 1)
				if self.photo_state:
					self.cc.go_straight(100, 0.5)
					if self.find_target:
						collide = True
		if self.find_target or collide:
			print "Find target, try to find door"
			if self.start_tag == 'A':
				direct = 'CCW'
			else:
				direct = 'CW'
			ts = time.time()
			count = 0
			while time.time() - ts <= 10:
				self.cc.rotate_in_place(direct, 100, 0.5)
				count += 1
				if count == 3:
					print "Reach 3 times"
					self.cc.go_straight(120, 0.5)
				if self.near_door:
					print "Find door, move toward it"
					self.cc.go_straight(130, 3)
					break
		self.first = False 
		print "Not found in first search, start try randomly search the map..."

	def traverse(self):
		while not rospy.is_shutdown():
			if self.first:
				self.approach_ball()
			else:
				self.cc.go_straight(100, 2)
				if self.photo_state:
					print "Find light, move toward it"
					self.cc.go_straight(120, 1)
					if self.find_target:
						print "Find target, try to find door"
						self.try_find_door()
				self.check_collision()
				r = random.randint(1, 3)
				direc_rand = random.randint(0, 1)
				if direc_rand == 0:
					direct = 'CW'
				else:
					direct = 'CCW'
				print "Rotate in place " + str(r) + " times"
				for i in range(r):
					self.cc.rotate_in_place(direct, 100, 0.5)
					if self.photo_state:
						print "Break since find light"
						break
					self.check_collision()
				if self.photo_state:
					print "Move toward light"
					self.cc.stop_moving(0.3)
					self.cc.go_straight(130, 1)
					if self.find_target:
						print "Find light, try to find door"
						self.try_find_door()

	def check_collision(self):
		if self.l_collision:
			print "Left collision"
			self.cc.stop_moving(1)
			self.cc.reverse(1)
			#self.cc.rotate_in_place('CW', 90, 1)
			self.cc.turn('Right', 2)
		if self.r_collision:
			print "Right collision"
			self.cc.stop_moving(1)
			self.cc.reverse(1)
			#self.cc.rotate_in_place('CCW', 90, 1)
			self.cc.turn('Left', 2)
		if self.ph_collision:
			print "Photo collision"
			self.cc.stop_moving(1)
			self.find_target = True
		if not self.l_collision and not self.r_collision:
			self.no_collide += 1
		if self.no_collide == 5:
			print "No collision too long time, car may be block"
			self.collide = 0
			self.cc.reverse(3)
			
	
	def try_find_door(self):
		count = 0
		while count < 3 and not self.near_door:
			self.cc.rotate_in_place('CW', 100, 0.5)
			count += 1
		if self.near_door:
			self.cc.go_straight(120, 1)
		
			
	def shutdown(self):
		print "Shutdown in progress"
		self.thread_l._Thread__stop()
		self.thread_r._Thread__stop()
		self.thread_p._Thread__stop()
		self.thread_ph._Thread__stop()
		self.thread_ir._Thread__stop()


if __name__ == "__main__":
	rospy.init_node("demo_4_node")
	d = Demo()
	rospy.on_shutdown(d.shutdown)
	c = raw_input("Press any key to start: ")
	d.start_thread()
	d.traverse()
	rospy.spin()
