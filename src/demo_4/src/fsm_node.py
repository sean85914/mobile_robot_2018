#!/usr/bin/env python

import random
import rospy
import smach
import smach_ros
import RPi.GPIO as GPIO

from std_msgs.msg import Bool
from car_control import Car_control

global controller
controller = Car_control()

'''
  Touch sensor, down : 27
  Touch sensor, left : 22
  Touch sensor, right:  5
  Photo sensor       : 21
  IR    sensor       : 20
'''
# Pin mode setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(27, GPIO.IN) # photo collision
GPIO.setup(22, GPIO.IN) # left collision
GPIO.setup(5, GPIO.IN) # right collision
GPIO.setup(21, GPIO.IN) # photo sensor
GPIO.setup(20, GPIO.IN) # IR sensor

class Go_straight(smach.State):
	def __init__(self, start_time):
		smach.State.__init__(self, outcomes=['right_collision', 'left_collision', 'photo_collision', 'time_out', 'keep_going'])
		self.controller = controller
		self.start_time = start_time.to_sec()
		self.times           = 0
	def execute(self, userdata):
		rospy.loginfo("Execute: %s", rospy.Time.now().to_sec() - self.start_time)
		if rospy.Time.now().to_sec() - self.start_time > 240:
			self.controller.stop_moving()
			return 'time_out'
		if GPIO.input(5):
			return 'right_collision'
		if self.times == 5: # go straight too long time
			self.times = 0
			rospy.loginfo("Go straight too long time, the car maybe blocked")
			rand = random.randint(0, 1)
			if rand == 0:
				return 'left_collision'
			else:
				return 'right_collision'
		if GPIO.input(22):
			self.controller.stop_moving()
			return 'left_collision'
		if GPIO.input(27):
			self.controller.stop_moving()
			return 'photo_collision'
		if not GPIO.input(21):
			self.controller.go_faster()
			rospy.loginfo("Find light")
			return 'keep_going'
		else:
			self.controller.go_straight()
			self.times += 1
			return 'keep_going'
		
class Left_collision_recovery(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['left_collision', 'right_collision', 'photo_collision', 'go_straight', 'time_out'])
		self.controller = controller
		self.first_execute   = True
		self.first_time      = None
		self.str             = None
		self.time            = None
	def execute(self, userdata):
		if self.first_execute:
			self.str = self.controller.left_collision_recovery()
			self.first_time = rospy.Time.now().to_sec()
			print "Left record time: ", self.first_time
			self.first_execute = False
			if self.str == 'spin':
				self.time = 0.2
			else:
				self.time = 1.0
		if rospy.Time.now().to_sec() - self.first_time > self.time:
			self.first_execute = True
			self.controller.stop_moving()
			return 'time_out'
		if GPIO.input(22):
			self.first_execute = True
			self.controller.stop_moving()
			return 'left_collision'
		if GPIO.input(5):	
			self.first_execute = True
			self.controller.stop_moving()
			return 'right_collision'
		if GPIO.input(27):
			self.controller.stop_moving()
			self.first_execute = True
			return 'photo_collision'
		if not GPIO.input(21):
			self.first_execute = True
			self.controller.stop_moving()
			return 'go_straight'
		rospy.sleep(0.1)
		return 'left_collision'
	
class Right_collision_recovery(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['left_collision', 'right_collision', 'photo_collision', 'go_straight', 'time_out'])
		self.controller = controller
		self.first_execute   = True
		self.first_time      = None
		self.str             = None
		self.time            = None
	def execute(self, userdata):
		if self.first_execute:
			self.str = self.controller.right_collision_recovery()
			self.first_time = rospy.Time.now().to_sec()
			print "Right record time: ", self.first_time
			self.first_execute = False
			if self.str == 'spin':
				self.time = 0.2
			else:
				self.time = 1.0
		if rospy.Time.now().to_sec() - self.first_time > self.time:
			self.first_execute = True
			self.controller.stop_moving()
			return 'time_out'
		if GPIO.input(22):
			self.first_execute = True
			self.controller.stop_moving()
			return 'left_collision'
		if GPIO.input(5):
			self.first_execute = True
			self.controller.stop_moving()
			return 'right_collision'
		if not GPIO.input(21):
			self.first_execute = True
			self.controller.stop_moving()
			return 'go_straight'
		if GPIO.input(27):
			self.controller.stop_moving()
			self.first_execute = True
			return 'photo_collision'
		rospy.sleep(0.1)
		return 'right_collision'

# Check if given data in range [up, low]
# Parameters:
# 	data: given data to check if in range
# 	up  : upper bound
# 	low : lower bound
def in_range(data, up, low):
	if data > low and data < up:
		return True
	else:
		return False

class Find_door(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['continue', 'goal'])
		self.controller = controller
		self.bound = [0.23, 0.16, 0.33, 0.26] # [0.22, 0.17] -> 1500, [0.27, 0.32] -> 600
		self.target_door = rospy.get_param("~target_door", 600) # target door frequency
	def execute(self, userdata):
		if GPIO.input(22):
			self.controller.left_collision_recovery_with_ball()
		if GPIO.input(5):
			self.controller.right_collision_recovery_with_ball()
		arr = []
		ts = rospy.Time.now().to_sec()
		while rospy.Time.now().to_sec() - ts < 0.2:
			arr.append(GPIO.input(20))
		data_len = len(arr)
		data_0_len = 0
		for i in range(data_len):
			if arr[i] == 0:
				data_0_len = data_0_len + 1
		ratio = data_0_len / float(data_len)
		print "ratio: ", ratio
		if in_range(ratio, self.bound[0], self.bound[1]):
			front = 1500
			print "Door 1500 in front of me"
		elif in_range(ratio, self.bound[2], self.bound[3]):
			front = 600
			print "Door 600 in front of me"
		else:
			front = None
			self.controller.rotate_heading()
			self.controller.stop_moving()
			return 'continue'
		if front == self.target_door:
			self.controller.go_toward_door()
			self.controller.stop_moving()
			return 'continue'
		else:
			self.controller.rotate_heading()
			self.controller.stop_moving()
			return 'continue'
		
def main():
	rospy.init_node("car_motion_fsm_node")
	# create a SMACH state machine
	sm = smach.StateMachine(outcomes=['SUCCEED', 'FAILED'])
	c = raw_input("Press any key to start")
	# open container
	start_time = rospy.Time.now()
	with sm:
 		smach.StateMachine.add(
			'Go_straight', 
			Go_straight(start_time),
			transitions={'right_collision':'Right_collision_recovery',
				     'left_collision' :'Left_collision_recovery',
				     'time_out':'FAILED',
                		     'keep_going': 'Go_straight',
				     'photo_collision': 'Find_door'}
		)
		smach.StateMachine.add(
			'Right_collision_recovery', 
			Right_collision_recovery(),
			transitions={'right_collision': "Right_collision_recovery",
				     'left_collision' :  "Left_collision_recovery",
				     'photo_collision': "Find_door", 
				     'go_straight' : "Go_straight",
				     'time_out':'Go_straight'}
		)
		smach.StateMachine.add(
			'Left_collision_recovery', 
			Left_collision_recovery(),
			transitions={'right_collision': "Right_collision_recovery",
				     'left_collision' :  "Left_collision_recovery",
				     'photo_collision': "Find_door",
				     'go_straight' : "Go_straight",
				     'time_out':'Go_straight'}
		)
		smach.StateMachine.add(
			'Find_door',
			Find_door(),
			transitions={'goal':'SUCCEED', 'continue':"Find_door"}
		)

	sis = smach_ros.IntrospectionServer('server_name', sm, '/MY_FSM')
	sis.start()
	
	# execute the state machine
	outcome = sm.execute()
	#rospy.spin()
	while sm.is_running():
		pass
	# stop the application
	sis.stop()
	global controller
	del controller
	
if __name__ == '__main__':
	main()
