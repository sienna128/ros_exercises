#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from ros_exercises.msg import OpenSpace
import math

def callback(data):
    #pub_d = rospy.Publisher('open_space/distance', Float32)
    #pub_a = rospy.Publisher('open_space/angle', Float32)
    pub_t = rospy.get_param("publisher_topic", "open_space")

    pub_m = rospy.Publisher(pub_t, OpenSpace)
    msg = OpenSpace()

    ls = data
    r = ls.ranges

    l = 0
    ang = None

    for i, d in enumerate(r):
        if d > l:
            l = d
            ang = ((-1.0 * 2.0 / 3.0) * math.pi) + (float(i) * (math.pi * 1.0 / 300.0))

    
    msg.angle = ang
    msg.distance = l

    rospy.loginfo(msg)
    pub_m.publish(msg)


    #rospy.loginfo(l)
    #rospy.loginfo(ang)
    #pub_d.publish(l)
    #pub_a.publish(ang)    

def listener():
    #Initialize node
    rospy.init_node('open_space_publisher', anonymous = True)

    sub = rospy.get_param("subscriber_topic", "fake_scan")
    rate = rospy.Rate(20) # 20hz

    while not rospy.is_shutdown():
        rospy.Subscriber(sub, LaserScan, callback)

        rospy.spin()

        rate.sleep()


if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
