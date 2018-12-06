#!/usr/bin/env python

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
'''
# Pin mode setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(27, GPIO.OUT) # photo collision
GPIO.setup(22, GPIO.OUT) # left collision
GPIO.setup(5, GPIO.OUT) # right collision

class Go_straight(smach.State):
	def __init__(self, start_time):
		smach.State.__init__(self, outcomes=['right_collision', 'left_collision', 'photo_collision', 'time_out', 'keep_going'])
		self.controller = controller
		self.sub_photo_state = rospy.Subscriber('photo_state', Bool,
                                                        self.photo_state_cb, queue_size = 1)
		self.start_time = start_time.to_sec()
		self.photo_state     = False
		self.times           = 0
	def execute(self, userdata):
		rospy.loginfo("Execute: %s", rospy.Time.now().to_sec() - self.start_time)
		if rospy.Time.now().to_sec() - self.start_time > 120:
			self.controller.stop_moving()
			return 'time_out'
		if GPIO.input(5):
			return 'right_collision'
		if self.times == 5: # go straight too long time
			self.times = 0
			rospy.loginfo("Go straight too long time, the car maybe blocked, go to left collision recovery")
			return 'left_collision'
		if GPIO.input(22):
			return 'left_collision'
		if GPIO.input(27):
			self.controller.stop_moving()
			return 'photo_collision'
		if self.photo_state:
			self.controller.go_faster()
			rospy.loginfo("Find light")
			return 'keep_going'
		else:
			self.controller.go_straight()
			self.times += 1
			return 'keep_going'
	def photo_state_cb(self, msg):
		self.photo_state = msg.data
		
class Left_collision_recovery(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['left_collision', 'right_collision', 'photo_collision', 'go_straight', 'time_out'])
		self.controller = controller
		self.sub_photo_state = rospy.Subscriber('photo_state', Bool,
							self.photo_state_cb, queue_size = 1)
		self.photo_state     = False
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
			return 'time_out'
		if GPIO.input(22):
			self.first_execute = True
			return 'left_collision'
		if GPIO.input(5):	
			self.first_execute = True
			return 'right_collision'
		if GPIO.input(27):
			self.controller.stop_moving()
			self.first_execute = True
			return 'photo_collision'
		if self.photo_state:
			self.first_execute = True
			return 'go_straight'
		rospy.sleep(0.1)
		return 'left_collision'
	def photo_state_cb(self, msg):
		self.photo_state = msg.data
	
class Right_collision_recovery(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['left_collision', 'right_collision', 'photo_collision', 'go_straight', 'time_out'])
		self.controller = controller
		self.sub_photo_state = rospy.Subscriber('photo_state', Bool,
							self.photo_state_cb, queue_size = 1)
		self.photo_state     = False
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
			return 'time_out'
		if GPIO.input(22):
			self.first_execute = True
			return 'left_collision'
		if GPIO.input(5):
			self.first_execute = True
			return 'right_collision'
		if self.photo_state:
			self.first_execute = True
			return 'go_straight'
		if GPIO.input(27):
			self.controller.stop_moving()
			self.first_execute = True
			return 'photo_collision'
		rospy.sleep(0.1)
		return 'right_collision'
	def photo_state_cb(self, msg):
		self.photo_state = msg.data
		
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
				     'photo_collision': 'SUCCEED'}
		)
		smach.StateMachine.add(
			'Right_collision_recovery', 
			Right_collision_recovery(),
			transitions={'right_collision': "Right_collision_recovery",
				     'left_collision' :  "Left_collision_recovery",
				     'photo_collision': "SUCCEED", 
				     'go_straight' : "Go_straight",
				     'time_out':'Go_straight'}
		)
		smach.StateMachine.add(
			'Left_collision_recovery', 
			Left_collision_recovery(),
			transitions={'right_collision': "Right_collision_recovery",
				     'left_collision' :  "Left_collision_recovery",
				     'photo_collision': "SUCCEED",
				     'go_straight' : "Go_straight",
				     'time_out':'Go_straight'}
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
