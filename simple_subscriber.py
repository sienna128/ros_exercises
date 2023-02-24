#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
import math

def callback(data):
    pub = rospy.Publisher('random_float_log', Float32, queue_size = 10)
    log_num = math.log(data.data)
    rospy.loginfo(log_num)
    pub.publish(log_num)
    

def listener():
    #Initialize node
    rospy.init_node('simple_subscriber', anonymous = True)


    rate = rospy.Rate(20) # 20hz

    while not rospy.is_shutdown():
        rospy.Subscriber('my_random_float', Float32, callback)

        rospy.spin()

        rate.sleep()


if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
