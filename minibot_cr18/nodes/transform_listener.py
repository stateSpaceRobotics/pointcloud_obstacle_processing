#!/usr/bin/env python  
import rospy

import math
import tf2_ros
import geometry_msgs.msg

if __name__ == '__main__':
    rospy.init_node('transform_listener')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    vive_pose = rospy.Subscriber('/device01', geometry_msgs.msg.Pose)
    # rospy.wait_for_service('spawn')
    # spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    # turtle_name = rospy.get_param('turtle', 'turtle2')
    # spawner(4, 2, 0, turtle_name)

    tf_pose = rospy.Publisher('/tf_pose', geometry_msgs.msg.Pose, queue_size=1)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('tracking', 'tf_tracking', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        msg = geometry_msgs.msg.Pose()
	msg.position.x = vive_pose.position.x + trans.transform.translation.x
	msg.position.y = vive_pose.position.y + trans.transform.translation.y
	msg.position.z = vive_pose.position.z + trans.transform.translation.z
	msg.orientation.x = vive_pose.orientation.x + trans.transform.rotation.x
	msg.orientation.y = vive_pose.orientation.y + trans.transform.rotation.y
	msg.orientation.z = vive_pose.orientation.z + trans.transform.rotation.z
	msg.orientation.w = vive_pose.orientation.w + trans.transform.rotation.w
#        msg.angular.z = 4 * math.atan2(trans.transform.translation.y, trans.transform.translation.x)
#        msg.linear.x = 0.5 * math.sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2)

        tf_pose.publish(msg)

        rate.sleep()
