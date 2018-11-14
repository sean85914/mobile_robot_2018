#!/usr/bin/env python

import rospy
import smach
import smach_ros

from std_msgs.msg import Bool
from car_control import Car_control

global controller
controller = Car_control()

class Go_straight(smach.State):
	def __init__(self, start_time):
		smach.State.__init__(self, outcomes=['right_collision', 'left_collision', 'photo_collision', 'time_out', 'keep_going'])
		self.controller = controller
		self.sub_right_collision = rospy.Subscriber('right_collision', Bool,
		                                            self.right_collision_cb, queue_size = 1)
		self.sub_left_collision = rospy.Subscriber('left_collision', Bool,
		                                           self.left_collision_cb, queue_size = 1)
		self.sub_photo_collision = rospy.Subscriber('photo_collision', Bool,
							    self.photo_collision_cb, queue_size = 1)
		self.sub_photo_state = rospy.Subscriber('photo_state', Bool,
                                                        self.photo_state_cb, queue_size = 1)
		self.start_time = start_time.to_sec()
		self.right_collision = False
		self.left_collision  = False
		self.photo_collision = False
		self.photo_state     = False
		self.times           = 0
	def execute(self, userdata):
		rospy.loginfo("Execute: %s", rospy.Time.now().to_sec() - self.start_time)
		if rospy.Time.now().to_sec() - self.start_time > 90:
			self.controller.stop_moving()
			return 'time_out'
		if self.right_collision:
			return 'right_collision'
		if self.times == 5: # go straight too long time
			self.times = 0
			rospy.loginfo("Go straight too long time, the car maybe blocked, go to left collision recovery")
			return 'left_collision'
		if self.left_collision:
			return 'left_collision'
		if self.photo_collision:
			self.controller.stop_moving()
			return 'photo_collision'
		else:
			self.controller.go_straight()
			self.times += 1
			return 'keep_going'
	def right_collision_cb(self, msg):
		self.right_collision = msg.data
	def left_collision_cb(self, msg):
		self.left_collision = msg.data
	def photo_collision_cb(self, msg):
		self.photo_collision = msg.data
	def photo_state_cb(self, msg):
		self.photo_state = msg.data
		
class Left_collision_recovery(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['left_collision', 'right_collision', 'photo_collision', 'go_straight', 'time_out'])
		self.controller = controller
		self.sub_left_collision  = rospy.Subscriber('left_collision', Bool,
							   self.left_collision_cb, queue_size = 1)
		self.sub_right_collision = rospy.Subscriber('right_collision', Bool,
							    self.right_collision_cb, queue_size = 1)
		self.sub_photo_collision = rospy.Subscriber('photo_collision', Bool,
							    self.photo_collision_cb, queue_size = 1)
		self.sub_photo_state = rospy.Subscriber('photo_state', Bool,
							self.photo_state_cb, queue_size = 1)
		self.left_collision  = False
		self.right_collision = False
		self.photo_collision = False
		self.photo_state     = False
		self.first_execute   = True
		self.first_time      = None
	def execute(self, userdata):
		if self.first_execute:
			self.controller.left_collision_recovery()
			self.first_time = rospy.Time.now().to_sec()
			print "Left record time: ", self.first_time
			self.first_execute = False
		if rospy.Time.now().to_sec() - self.first_time > 1.0:
			self.first_execute = True
			return 'time_out'
		if self.left_collision:
			self.first_execute = True
			return 'left_collision'
		if self.right_collision:	
			self.first_execute = True
			return 'right_collision'
		if self.photo_collision:
			self.controller.stop_moving()
			self.first_execute = True
			return 'photo_collision'
		if self.photo_state:
			self.first_execute = True
			return 'go_straight'
		return 'left_collision'
	def left_collision_cb(self, msg):
		self.left_collision = msg.data
	def right_collision_cb(self, msg):
		self.right_collision = msg.data
	def photo_collision_cb(self, msg):
		self.photo_collision = msg.data
	def photo_state_cb(self, msg):
		self.photo_state = msg.data
	
class Right_collision_recovery(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['left_collision', 'right_collision', 'photo_collision', 'go_straight', 'time_out'])
		self.controller = controller
		self.sub_left_collision  = rospy.Subscriber('left_collision', Bool,
							    self.left_collision_cb, queue_size = 1)
		self.sub_right_collision = rospy.Subscriber('right_collision', Bool,
							    self.right_collision_cb, queue_size = 1)
		self.sub_photo_collision = rospy.Subscriber('photo_collision', Bool,
							    self.photo_collision_cb, queue_size = 1)
		self.sub_photo_state     = rospy.Subscriber('photo_state', Bool,
							    self.photo_state_cb, queue_size = 1)
		self.left_collision  = False
		self.right_collision = False
		self.photo_collision = False
		self.photo_state     = False
		self.first_execute   = True
		self.first_time      = None            
	def execute(self, userdata):
		if self.first_execute:
			self.controller.right_collision_recovery()
			self.first_time = rospy.Time.now().to_sec()
			print "Right record time: ", self.first_time
			self.first_execute = False
		if rospy.Time.now().to_sec() - self.first_time > 1.0:
			self.first_execute = True
			return 'time_out'
		if self.left_collision:
			self.first_execute = True
			return 'left_collision'
		if self.right_collision:
			self.first_execute = True
			return 'right_collision'
		if self.photo_state:
			self.first_execute = True
			return 'go_straight'
		if self.photo_collision:
			self.controller.stop_moving()
			self.first_execute = True
			return 'photo_collision'
		return 'right_collision'
	def left_collision_cb(self, msg):
		self.left_collision = msg.data
	def right_collision_cb(self, msg):
		self.right_collision = msg.data
	def photo_collision_cb(self, msg):
		self.photo_collision = msg.data
	def photo_state_cb(self, msg):
		self.photo_state = msg.data
		
def main():
	rospy.init_node("car_motion_fsm_node")
	# create a SMACH state machine
	sm = smach.StateMachine(outcomes=['SUCCEED', 'FAILED'])
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
