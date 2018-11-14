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
		smach.State.__init__(self, outcomes=['right_collision', 
                                             'left_collision',
                                             'photo_collision',
                                             'time_out',
                                             'keep_going'])
		self.controller = controller
		self.sub_right_collision = rospy.Subscriber('right_collision', Bool,
		                                            self.right_collision_cb, queue_size = 10)
		self.sub_left_collision = rospy.Subscriber('left_collision', Bool,
		                                           self.left_collision_cb, queue_size = 10)
		self.sub_photo_state = rospy.Subscriber('photo_state', Bool,
                                                self.photo_state_cb, queue_size = 10)
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
		if self.times == 8: # go straight too long time
			self.times = 0
			rospy.loginfo("Go straight too long time, the car maybe blocked, go to right collision recovery")
			return 'right_collision'
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
	def photo_state_cb(self, msg):
		self.photo_state = msg.data
		
class Left_collision_recovery(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['time_out'])
		self.controller = controller
	def execute(self, userdata):
		self.controller.left_collision_recovery()
		rospy.sleep(3.0)
		return 'time_out'
	
class Right_collision_recovery(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['time_out'])
		self.controller = controller
	def execute(self, userdata):
		self.controller.right_collision_recovery()
		rospy.sleep(3.0)
		return 'time_out'

class Try_find_light(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['find', 'not_found'])
		self.controller = controller
		self.photo_collision = False
		self.sub_photo_collision = rospy.Subscriber('photo_collision', Bool,
         											self.photo_collision_cb, queue_size = 10)
	def execute(self, userdata):
		self.start_time = rospy.Time.now().to_sec()
		while rospy.Time.now().to_sec() - self.start_time < 3.0:
			if self.photo_collision:
				return 'find'
		rospy.loginfo("Detected photo but not attach it")
		return 'not_found'

	def photo_collision_cb(self, msg):
		self.photo_collision = msg.data
		
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
				'left_collision':'Left_collision_recovery',
				'photo_detected':'Try_find_light',
				'time_out':'FAILED',
                'keep_going': 'Go_straight'}
		)
		smach.StateMachine.add(
			'Right_collision_recovery', 
			Right_collision_recovery(),
			transitions={'time_out':'Go_straight'}
		)
		smach.StateMachine.add(
			'Left_collision_recovery', 
			Left_collision_recovery(),
			transitions={'time_out':'Go_straight'}
		)
		smach.StateMachine.add(
			'Try_find_light',
			Try_find_light(),
			transition={'find':'SUCCEED', 'not_find':'Go_straight'}
		)

	sis = smach_ros.IntrospectionServer('server_name', sm, '/MY_FSM')
	sis.start()
	
	# execute the state machine
	outcome = sm.execute()
	rospy.spin()
	# stop the application
	sis.stop()
	
if __name__ == '__main__':
	main()
