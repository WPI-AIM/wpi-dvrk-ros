#!/usr/bin/env python
import rospy
import roslib
from geometry_msgs.msg import WrenchStamped, Wrench
from geometry_msgs.msg import PoseStamped, Pose
import tf
from tf import Transfrom
from tf import Quaternion
from tf import Vector3

cur_pose = Pose()
wrench = WrenchStamped()

def pose_cb(data):
    global cur_pose
    cur_pose = data.Pose()
    pass

def wrench_cb(data):
    global wrench
    wrench.wrench = data
    wrench.header.frame_id = "MTMR_wrist_roll_link"
    pass


def init():
    global wrench
    rospy.init_node("wrench_pub")
    sub = rospy.Subscriber("/dvrk/MTMR/set_wrench_body", Wrench, wrench_cb)
    sub2 = rospy.Subscriber("/dvrk/MTMR/position_cartesian_current", PoseStamped, pose_cb)
    pub = rospy.Publisher("/dvrk/MTMR/stamped_body_wrench", WrenchStamped, queue_size=10)

    rate = rospy.Rate(50)

    quat = Quaternion()
    fvec = Vector3()
    cfvec = Vector3()

    while not rospy.is_shutdown():
        quat.setX = cur_pose.orientation.x
        quat.setY = cur_pose.orientation.y
        quat.setZ = cur_pose.orientation.z
        quat.setW = cur_pose.orientation.w
        fvec.setValue(cur_pose.position.x,cur_pose.position.y,cur_pose.position.z)
        cfvec = quat * fvec
        wrench.wrench.force.x = cfvec.x()
        wrench.wrench.force.y = cfvec.y()
        wrench.wrench.force.z = cfvec.z()
        pub.publish(wrench)
        rate.sleep()

if __name__ == '__main__':
    init()
