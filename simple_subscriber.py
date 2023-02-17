#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
import math

def callback(data):
    pub = rospy.Publisher('random_float_log', Float32)
    log_num = math.log(data.data)
    rospy.loginfo(log_num)
    pub.publish(log_num)
    

def listener():
    #Initialize node
    rospy.init_node('simple_subscriber', anonymous = True)

    rospy.Subscriber('my_random_float', Float32, callback)

    rospy.spin()


if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
