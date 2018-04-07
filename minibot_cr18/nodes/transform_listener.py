#!/usr/bin/env python  
import rospy

import math
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs.msg
import nav_msgs.msg

def vive_pose_callback(data):
    try:
        trans = tfBuffer.lookup_transform('world', 'odom', rospy.Time(0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rate.sleep()
        # continue
    
    # take in vive pose, multiply by transform, then publish back out
    transformed_pose = nav_msgs.msg.Odometry()
    transformed_pose.header.stamp = rospy.Time.now()
    transformed_pose.header.frame_id = "world"
    transformed_pose.child_frame_id = "device01"
    transformed_pose.pose.covariance = data.pose.covariance
    transformed_pose.twist = data.twist
    transformed_pose.pose.pose = tf2_geometry_msgs.do_transform_pose(data.pose, trans).pose
    tf_pose.publish(transformed_pose)

if __name__ == '__main__':
    rospy.init_node('transform_listener')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    vive_pose = rospy.Subscriber('/device01', nav_msgs.msg.Odometry, vive_pose_callback)
    # rospy.wait_for_service('spawn')
    # spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    # turtle_name = rospy.get_param('turtle', 'turtle2')
    # spawner(4, 2, 0, turtle_name)

    tf_pose = rospy.Publisher('/tf_pose', nav_msgs.msg.Odometry, queue_size=1)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        # try:
        #     trans = tfBuffer.lookup_transform('tracking', 'tf_tracking', rospy.Time())
        # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        #     rate.sleep()
        #     continue

        rate.sleep()
