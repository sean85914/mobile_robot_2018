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
		                                            self.right_collision_cb, queue_size = 1)
		self.sub_left_collision = rospy.Subscriber('left_collision', Bool,
		                                           self.left_collision_cb, queue_size = 1)
		self.sub_photo_collision = rospy.Subscriber('photo_collision', Bool,
                                                    self.photo_collision_cb, queue_size = 1)
		self.start_time = start_time.to_sec()
		self.right_collision = False
		self.left_collision  = False
		self.photo_collision = False                                          
	def execute(self, userdata):
		if rospy.Time.now().to_sec() - self.start_time > 90:
			self.controller.stop()
			return 'time_out'
		if self.right_collision:
			return 'right_collision'
		if self.left_collision:
			return 'left_collision'
		if self.photo_collision:
			self.controller.stop()
			return 'photo_collision'
		else:
			self.controller.go_straight()
			#rospy.sleep(3.0)
			return 'keep_going'
	def right_collision_cb(self, msg):
		self.right_collision = msg.data
	def left_collision_cb(self, msg):
		self.left_collision = msg.data
	def photo_collision_cb(self, msg):
		self.photo_collision = msg.data
		
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
				'photo_collision':'SUCCEED',
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

	sis = smach_ros.IntrospectionServer('server_name', sm, '/MY_FSM')
	sis.start()
	
	# execute the state machine
	outcome = sm.execute()
	
	rospy.spin()
	# stop the application
	sis.stop()
	
if __name__ == '__main__':
	main()
