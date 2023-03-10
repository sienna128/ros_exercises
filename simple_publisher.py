#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
import random

def talker():
    #Initialize the node
    pub = rospy.Publisher('my_random_float', Float32, queue_size = 10)
    rospy.init_node("simple_publisher", anonymous = True)

    rate = rospy.Rate(20) # 20hz

    while not rospy.is_shutdown():
        num = random.random() * 10
        rospy.loginfo(num)
        pub.publish(num)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

    

    
