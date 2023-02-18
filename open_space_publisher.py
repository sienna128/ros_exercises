#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
import math

def callback(data):
    pub_d = rospy.Publisher('open_space/distance', Float32)
    pub_a = rospy.Publisher('open_space/angle', Float32)
    ls = data
    r = ls.ranges

    l = 0
    ang = None

    for i, d in enumerate(r):
        if d > l:
            l = d
            ang = ((-1.0 * 2.0 / 3.0) * math.pi) + (float(i) * (math.pi * 1.0 / 300.0))

    

    rospy.loginfo(l)
    rospy.loginfo(ang)
    pub_d.publish(l)
    pub_a.publish(ang)    

def listener():
    #Initialize node
    rospy.init_node('open_space_publisher', anonymous = True)

    rospy.Subscriber('fake_scan', LaserScan, callback)

    rospy.spin()


if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
