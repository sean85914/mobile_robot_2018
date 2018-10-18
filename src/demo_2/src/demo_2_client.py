#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16

def main():
    rospy.init_node('demo_2')
    rospy.sleep(3.0)
    pub_r = rospy.Publisher("right_pwm", Int16, queue_size = 10)
    pub_l = rospy.Publisher("left_pwm",  Int16, queue_size = 10)
    trim = rospy.get_param("/trim", 0)
    trim = int(trim)
    while not rospy.is_shutdown():
        trim = rospy.get_param("/trim")
        trim = int(trim)
        try:
            pwm_r = input("user's right:")
            pwm_l = input("user's left:")
            if(pwm_r != 0 ):
                pwm_r = pwm_r + trim
            if(pwm_l != 0 ):
                pwm_l = pwm_l - trim
            pub_r.publish(Int16(pwm_r + trim))
            pub_l.publish(Int16(pwm_l - trim))
            print trim
        except NameError:
            pass

if __name__ == "__main__":
    main()
