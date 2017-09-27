#!/usr/bin/env python

import rospy
import roslib
from geometry_msgs.msg import PoseStamped, Pose
import tf
import tf.transformations

cur_pose = Pose()

def psm_pose_cb(data):
    global cur_pose
    cur_pose = data.pose
    pass


def init():
    rospy.init_node("dvrk_psm_debug")
    psm_pose_sub = rospy.Subscriber("/dvrk/PSM1/position_cartesian_current", PoseStamped, psm_pose_cb)
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        br.sendTransform((cur_pose.position.x, cur_pose.position.y,cur_pose.position.z), (cur_pose.orientation.x,
                         cur_pose.orientation.y, cur_pose.orientation.z, cur_pose.orientation.w), rospy.Time.now(),
                         "psm_pose", "/world")
        rate.sleep()

if __name__ == '__main__':
    init()