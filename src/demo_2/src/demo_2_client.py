#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16

def main():
    rospy.init_node('demo_2')
    rospy.sleep(3.0)
    while not rospy.is_shutdown():
        pub_r = rospy.Publisher("right_pwm", Int16, queue_size = 10)
        pub_l = rospy.Publisher("left_pwm", Int16, queue_size = 10)
        pwm_r = input("user's right:")
        pwm_l = input("user's left:")
        if type(pwm_r) == int and type(pwm_l) == int:
            pub_r.publish(Int16(pwm_r))
            pub_l.publish(Int16(pwm_l))
    

if __name__ == "__main__":
    main()
