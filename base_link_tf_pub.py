#!/usr/bin/env python
import rospy
import tf2_ros
import numpy as np
import geometry_msgs.msg
import transformations



def callback(b, t, l2w):
    br = b 
    tfBuffer = t



    l2w_arr_t = np.array([[l2w.transform.translation.x],
                        [l2w.transform.translation.y],
                        [l2w.transform.translation.z]])

    l2w_arr_r = np.array([l2w.transform.rotation.x, l2w.transform.rotation.y, l2w.transform.rotation.z, l2w.transform.rotation.w])

    l_wrt_w_mat = transformations.quaternion_matrix(l2w_arr_r)

    b_wrt_l_mat = np.linalg.inv(np.array([[1, 0, 0, -0.05], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]))

    b_wrt_w_mat = np.dot(l_wrt_w_mat, b_wrt_l_mat)

    b_wrt_w_quat = transformations.quaternion_from_matrix(b_wrt_w_mat)

    #left cam
    w2l = geometry_msgs.msg.TransformStamped()
    w2l.header.stamp = rospy.Time.now()

    w2l.header.frame_id = "world"
    w2l.child_frame_id = "base_link_gt2"
    t = w2l.header.stamp.to_sec()

    w2l.transform.translation.x = b_wrt_w_mat[0,3] #l2w.transform.translation.x - 0.05
    w2l.transform.translation.y = b_wrt_w_mat[1,3]#l2w.transform.translation.y
    w2l.transform.translation.z = b_wrt_w_mat[2,3]#l2w.transform.translation.z

    w2l.transform.rotation.x = b_wrt_w_quat[0]
    w2l.transform.rotation.y = b_wrt_w_quat[1]
    w2l.transform.rotation.z = b_wrt_w_quat[2]
    w2l.transform.rotation.w = b_wrt_w_quat[3]

    br.sendTransform(w2l)


    
    
    
    # Use it to transform between frames
    #pose_transformed = tf2_geometry_msgs.do_transform_pose(pose, transform)


       

def listener():
    # Initialize the node
    rospy.init_node("base_link_tf_pub")

    # Create a broadcaster
    br = tf2_ros.TransformBroadcaster()

    # Publish messages at 10 hz
    r = rospy.Rate(10)

    # Create a listener
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    while not rospy.is_shutdown():
        g = False
        try:
        # Fetch the transform
            print("try call back")
            l2w = tfBuffer.lookup_transform("world", "left_cam", rospy.Time())
            callback(br, tfBuffer, l2w)
            print("finish call back")
            # Wait for the next event
            #g = True
            r.sleep()
            
        except:
            r.sleep()
            print("Except")
            continue
        

        

if __name__ == "__main__":
    listener()
