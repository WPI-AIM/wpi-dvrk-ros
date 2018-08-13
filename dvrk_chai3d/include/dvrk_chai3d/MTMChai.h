//===========================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2017-2018

    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.

    * Neither the name of CHAI3D nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.

    \author    amunawar@wpi.edu
    \author    Adnan Munawar
    \version   0.1
*/
//===========================================================================

#ifndef CDVRK_MTMH
#define CDVRK_MTMH

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Wrench.h"
#include "string.h"
#include "tf/tf.h"
#include "tf/LinearMath/Matrix3x3.h"

class DVRK_MTM{
public:
    DVRK_MTM();
    ~DVRK_MTM();

    bool home();

    void state_sub_cb(const std_msgs::StringConstPtr &msg);
    void pose_sub_cb(const geometry_msgs::PoseStampedConstPtr &msg);
    void joint_sub_cb(const sensor_msgs::JointStateConstPtr &msg);
    void clutch_sub_cb(const sensor_msgs::JoyConstPtr &msg);
    void init();
    void _rate_sleep();

    bool _is_mtm_available();
    bool _in_effort_mode();
    bool set_mode(std::string str);
    bool set_force(double fx, double fy, double fz);

    geometry_msgs::PoseStamped cur_pose, pre_pose;
    sensor_msgs::JointState cur_joint, pre_joint;
    std_msgs::String cur_state;
    geometry_msgs::Wrench cur_wrench, cmd_wrench;
    geometry_msgs::Quaternion gm_cur_ori;
    tf::Quaternion tf_cur_ori;
    tf::Matrix3x3 mat_ori, ori_corr;
    std_msgs::String state_cmd;
    void get_cur_position(double &x, double &y, double &z);

    std::string arm_name;
    std::string _m_effort_mode = "DVRK_EFFORT_CARTESIAN";
    float pos_offset[3];

private:
    ros::NodeHandle *n;
    ros::Publisher force_pub;
    ros::Publisher force_orientation_safety_pub;
    ros::Subscriber pose_sub;
    ros::Subscriber joint_sub;
    ros::Subscriber state_sub;
    ros::Subscriber clutch_sub;
    ros::Publisher state_pub;
    ros::Rate *rate;
    bool _clutch_pressed;
    double chai_origin[3];
    double scale;

};
#endif
