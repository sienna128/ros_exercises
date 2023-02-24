#!/usr/bin/env python
import rospy
import tf2_ros
import numpy as np
import geometry_msgs.msg
import transformations



def callback(b, t, b2w):
    br = b 
    tfBuffer = t


    b2w_arr_t = np.array([[b2w.transform.translation.x],
                        [b2w.transform.translation.y],
                        [b2w.transform.translation.z]])

    b2w_arr_r = np.array([b2w.transform.rotation.x, b2w.transform.rotation.y, b2w.transform.rotation.z, b2w.transform.rotation.w])

    b_wrt_w_mat = transformations.quaternion_matrix(b2w_arr_r)

    left_wrt_b_mat = np.array([[1, 0, 0, -0.05], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

    left_wrt_w_mat = np.dot(b_wrt_w_mat, left_wrt_b_mat)

    left_wrt_w_quat = transformations.quaternion_from_matrix(left_wrt_w_mat)

    #left cam
    w2l = geometry_msgs.msg.TransformStamped()
    w2l.header.stamp = rospy.Time.now()

    w2l.header.frame_id = "world"
    w2l.child_frame_id = "left_cam"
    t = w2l.header.stamp.to_sec()

    w2l.transform.translation.x = left_wrt_w_mat[0,3] #b2w.transform.translation.x - 0.05
    w2l.transform.translation.y = left_wrt_w_mat[1,3]#b2w.transform.translation.y
    w2l.transform.translation.z = left_wrt_w_mat[2,3]#b2w.transform.translation.z

    w2l.transform.rotation.x = left_wrt_w_quat[0]
    w2l.transform.rotation.y = left_wrt_w_quat[1]
    w2l.transform.rotation.z = left_wrt_w_quat[2]
    w2l.transform.rotation.w = left_wrt_w_quat[3]

    br.sendTransform(w2l)


    #right cam
    
    arr_r = np.array([[1, 0, 0, 0.1], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

    r2w_m = np.dot(left_wrt_w_mat, arr_r)
    r2w_q = transformations.quaternion_from_matrix(r2w_m)

    r2w = geometry_msgs.msg.TransformStamped()
    r2w.header.stamp = rospy.Time.now()

    r2w.header.frame_id = "left_cam"
    r2w.child_frame_id = "right_cam"
    t = r2w.header.stamp.to_sec()

    r2w.transform.translation.x = 0.1 #w2l.transform.translation.x + 0.1
    r2w.transform.translation.y = 0 #w2l.transform.translation.y
    r2w.transform.translation.z = 0 #w2l.transform.translation.z

    r2w.transform.rotation.x = 0 #r2w_q[0]
    r2w.transform.rotation.y = 0 #r2w_q[1]
    r2w.transform.rotation.z = 0 #r2w_q[2]
    r2w.transform.rotation.w = 1 #r2w_q[3]

    br.sendTransform(r2w)
    
    
    # Use it to transform between frames
    #pose_transformed = tf2_geometry_msgs.do_transform_pose(pose, transform)


       

def listener():
    # Initialize the node
    rospy.init_node("dynamic_tf_cam_publisher")

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
            b2w = tfBuffer.lookup_transform("world", "base_link_gt", rospy.Time())
            
            print("finish call back")
            # Wait for the next event
            g = True
            
        except:
            r.sleep()
            print("Except")
            continue
        
        if g:
            callback(br, tfBuffer, b2w)
            r.sleep()

        

if __name__ == "__main__":
    listener()
