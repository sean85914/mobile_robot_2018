#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16


def check_parameter(event):
    global trim
    trim_temp = trim
    trim = rospy.get_param("~trim")
    if(trim_temp != trim):
        rospy.loginfo("trim set from %d to %d" %(trim_temp, trim))

def main():
    rospy.init_node('client')
    print rospy.get_namespace()
    rospy.sleep(3.0)
    rospy.Timer(rospy.Duration(1.0), check_parameter)
    pub_r = rospy.Publisher("right_pwm", Int16, queue_size = 10)
    pub_l = rospy.Publisher("left_pwm",  Int16, queue_size = 10)
    global trim
    trim = rospy.get_param("~trim")
    trim = int(trim)
    while not rospy.is_shutdown():
        try:
            pwm_r = int(raw_input("user's right:"))
        except ValueError:
            rospy.logerr("Invalid input")
            pwm_r = 0
        try:
            pwm_l = int(raw_input("user's left:"))
        except ValueError:
            rospy.logerr("Invalid input")
            pwm_l = 0
        if(pwm_r != 0 ):
            pwm_r = pwm_r + trim
        if(pwm_l != 0 ):
            pwm_l = pwm_l - trim
        pub_r.publish(Int16(pwm_r))
        pub_l.publish(Int16(pwm_l))

if __name__ == "__main__":
    main()
