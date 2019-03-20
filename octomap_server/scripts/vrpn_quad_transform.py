#! /usr/bin/env python
"""
Author:         Carl Stahoviak
Date Created:   Jan 28, 2019
Last Edited:    Jan 28, 2019

Edited by Gene Rush

Task: To rpublish a transform from the base_link frame to the world frame,
such that the transform is completely described by the current pose publoshed
by the Vicon system on the topic: /vrpn_client_node/RadarQuad/pose

"""

import rospy
import tf

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2

transform = tf.TransformBroadcaster()

def pose_repub(msg):
    t = TransformStamped()
    t.child_frame_id = "base_link"
    t.header.stamp = rospy.Time(0)
    t.header.frame_id = msg.header.frame_id

    t.transform.translation.x = msg.pose.position.x
    t.transform.translation.y = msg.pose.position.y
    t.transform.translation.z = msg.pose.position.z

    t.transform.rotation.x = msg.pose.orientation.x
    t.transform.rotation.y = msg.pose.orientation.y
    t.transform.rotation.z = msg.pose.orientation.z
    t.transform.rotation.w = msg.pose.orientation.w

    transform.sendTransformMessage(t)


def main():
    
    rospy.init_node('vrpn_pose_tf')

    robot_name = rospy.get_param('~robot_name')
    rospy.loginfo("SCRIPT ROBOT NAME IS : %s", robot_name)

    pose_topic = "/vrpn_client_node/" + robot_name + "/pose"
    rospy.loginfo("SCRIPT POSE TOPIC IS : %s", pose_topic)
   
    pose_sub = rospy.Subscriber(pose_topic, PoseStamped, pose_repub)
    rospy.loginfo("Subscribing to %s", pose_topic)

    r = rospy.Rate(10)      # 10 Hz
    while not rospy.is_shutdown():
        r.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: # ctrl-c
        pass

    rospy.spin()
