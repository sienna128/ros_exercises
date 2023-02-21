#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import random
import math


def talker():
    #Initialize the node
    rospy.init_node("fake_scan_publisher", anonymous = True)

    #parameters

    p_topic = rospy.get_param("publish_topic", "fake_scan")
    p_rate = rospy.get_param("publish_rate", 20)
    ang_min = rospy.get_param("angle_min", (-1.0 * 2.0 / 3.0) * math.pi)
    ang_max = rospy.get_param("angle_max", (1.0 * 2.0 / 3.0) * math.pi)
    rang_min = rospy.get_param("range_min", 1.0)
    rang_max = rospy.get_param("range_max", 10.0)
    ang_inc = rospy.get_param("angle_increment", (1.0 / 3000.0) * math.pi)

    pub = rospy.Publisher(p_topic, LaserScan)

    rate = rospy.Rate(p_rate) # 20hz

    while not rospy.is_shutdown():
        

        ls = LaserScan()

        ls.header.frame_id = "base_link"
        ls.header.stamp = rospy.get_rostime()

        ls.angle_min = ang_min
        ls.angle_max = ang_max
        ls.angle_increment = ang_inc
        ls.scan_time = 1.0 / 20.0

        ls.range_min = rang_min
        ls.range_max = rang_max

        r = []
        
        s = 0
        for i in range(int(((ls.angle_max - ls.angle_min) / ls.angle_increment) + 1)):
            num = random.random() * 10
            r.append(num)

        ls.ranges = r

        rospy.loginfo(ls)
        pub.publish(ls)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
