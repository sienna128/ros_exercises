#!/usr/bin/env python
import rospy

# to get commandline arguments
import sys

# because of transformations
import tf

import tf2_ros
import geometry_msgs.msg

import numpy as np


if __name__ == "__main__":
    rospy.init_node('static_tf_cam_publisher')
    br = tf2_ros.StaticTransformBroadcaster()

    r = rospy.Rate(10)


    #left cam
    w2l = geometry_msgs.msg.TransformStamped()
    w2l.header.stamp = rospy.Time.now()

    w2l.header.frame_id = "base_link_gt"
    w2l.child_frame_id = "left_cam"
    t = w2l.header.stamp.to_sec()

    w2l.transform.translation.x = -0.05
    w2l.transform.translation.y = 0
    w2l.transform.translation.z = 0

    w2l.transform.rotation.x = 0
    w2l.transform.rotation.y = 0
    w2l.transform.rotation.z = 0
    w2l.transform.rotation.w = 1

    #right cam
    r2w = geometry_msgs.msg.TransformStamped()
    r2w.header.stamp = rospy.Time.now()

    r2w.header.frame_id = "base_link_gt"
    r2w.child_frame_id = "right_cam"
    t = r2w.header.stamp.to_sec()

    r2w.transform.translation.x = 0.05
    r2w.transform.translation.y = 0
    r2w.transform.translation.z = 0

    r2w.transform.rotation.x = 0
    r2w.transform.rotation.y = 0
    r2w.transform.rotation.z = 0
    r2w.transform.rotation.w = 1

    br.sendTransform([w2l, r2w])

    rospy.spin()
