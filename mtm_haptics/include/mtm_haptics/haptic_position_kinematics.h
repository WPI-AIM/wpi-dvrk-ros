#ifndef _haptic_position_kinematics_h
#define _haptic_position_kinematics_h
#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>

#include <cisstNumerical/nmrLSMinNorm.h>
#include <cisstRobot/robManipulator.h>
#include <cisstVector.h>
#include <cisstParameterTypes/prmForceCartesianSet.h>
#include <cisst_ros_bridge/mtsCISSTToROS.h>
#include <cisst_ros_bridge/mtsROSToCISST.h>

class MTM_pos_kinematics{
public:
    MTM_pos_kinematics();
    void compute_torques(const geometry_msgs::Wrench &F, sensor_msgs::JointState torque_msg);

protected:
    vctDoubleVec mtm_joint_current;
    vctDoubleVec mtm_joint_command;
    vctFrm4x4 mtm_pose_current;
    vctFrm4x4 mtm_pose_command;
    prmForceCartesianSet newForce;

    ros::Subscriber cur_jnt_sub;
    ros::NodeHandle node_;

    void jnt_pos_cb(const sensor_msgs::JointStateConstPtr &msg);

    robManipulator mtm_manip;
    robManipulator::Errno result;
};


#endif // _haptic_position_kinematics_h
