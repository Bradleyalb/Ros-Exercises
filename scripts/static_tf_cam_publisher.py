#!/usr/bin/env python
import roslib
import rospy
import numpy as np
import math
import tf2_ros
import tf
from geometry_msgs.msg import TransformStamped


def publish(from_name, to, rot_matrix):

    t = TransformStamped()

        # Read message content and assign it to
    # corresponding tf variables
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = from_name
    t.child_frame_id = to

    # Turtle only exists in 2D, thus we get x and y translation
    # coordinates from the message and set the z coordinate to 0
    left_trans = rot_matrix[:3,3]
    t.transform.translation.x = left_trans[0]
    t.transform.translation.y = left_trans[1]
    t.transform.translation.z = left_trans[2]

    # For the same reason, turtle can only rotate around one axis
    # and this why we set rotation in x and y to 0 and obtain
    # rotation in z axis from the message
    q = tf.transformations.quaternion_from_matrix(rot_matrix)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    return t


def publish_rotations(data):
    rot = data.transform.rotation
    rot = np.array([rot.x,rot.y,rot.z,rot.w])
    trans = data.transform.translation
    trans = np.array([[trans.x, trans.y, trans.z]]).T

    R = tf.transformations.quaternion_matrix(rot)

    left_cam_pos = np.array([[-0.5,0,0]]).T
    right_cam_to_left_pos = np.array([[-1,0,0]]).T
    identity_rotation = np.array([[1,0,0],[0,1,0],[0,0,1]])
    #4x4 matrix
    identity_trans = np.concatenate((identity_rotation, trans), axis = 1)
    base_link_to_world = np.concatenate((np.concatenate((identity_rotation, trans), axis = 1), np.array([[0,0,0,1]])), axis=0)
    #Left_cam_to_world
    left_cam_base_link = np.concatenate((np.concatenate((identity_rotation, left_cam_pos), axis = 1), np.array([[0,0,0,1]])), axis=0)
    #Right_cam_to_left
    right_cam_to_left = np.concatenate((np.concatenate((identity_rotation, right_cam_to_left_pos), axis = 1), np.array([[0,0,0,1]])), axis=0)
    left_cam_to_world = np.matmul(left_cam_base_link,base_link_to_world)

    t1 = publish('world', 'left_cam',left_cam_to_world)
    t2 = publish('left_cam', 'right_cam',right_cam_to_left)
    br = tf2_ros.StaticTransformBroadcaster()
    br.sendTransform([t1,t2])
if __name__ == '__main__':
    rospy.init_node('static_tf_cam_publisher')
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)

    rate = rospy.Rate(10.0)
    flag = False
    while not rospy.is_shutdown() and not flag:
        try:
            #Position transform, quanternion rotation)
            trans = buffer.lookup_transform('world', 'base_link_gt', rospy.Time(0))
            flag = True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            continue
        publish_rotations(trans)
        rate.sleep()

    rospy.spin()
