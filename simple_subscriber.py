#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
import math

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def listener():
    #Initialize node
    rospy.init_node('simple_subscriber', anonymous = True)

    rospy.Subscriber('my_random_float', Float32, callback)

    rospy.spin()

def talker():
    pub = rospy.Publisher('random_float_log', Float32)
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        log_num = math.log(5) #replace w real thing

        rospy.loginfo(log_num)
        pub.publish(log_num)
        rate.sleep()

if __name__ == '__main__':
    listener()

    try:
        talker()
    except rospy.ROSInterruptException:
        pass