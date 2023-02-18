#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import random
import math


def talker():
    #Initialize the node
    pub = rospy.Publisher('fake_scan', LaserScan)
    rospy.init_node("fake_scan_publisher", anonymous = True)

    rate = rospy.Rate(20) # 20hz

    while not rospy.is_shutdown():
        

        ls = LaserScan()

        ls.header.frame_id = "base_link"
        ls.header.stamp = rospy.get_rostime()

        ls.angle_min = (-1.0 * 2.0 / 3.0) * math.pi
        ls.angle_max = (1.0 * 2.0 / 3.0) * math.pi
        ls.angle_increment = (1.0 / 3000.0) * math.pi
        ls.scan_time = 1.0 / 20.0

        ls.range_min = 1.0
        ls.range_max = 10.0

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
