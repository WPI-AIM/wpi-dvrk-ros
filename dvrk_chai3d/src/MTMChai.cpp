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

#include "dvrk_chai3d/MTMChai.h"
DVRK_MTM::DVRK_MTM(){
}

void DVRK_MTM::init(){
    int argc;
    char** argv;
    ros::M_string s;
    ros::init(s, "my_node");

    n = new ros::NodeHandle;
    rate = new ros::Rate(1000);

    n->param(std::string("arm"), arm_name, std::string("MTMR"));

    pose_sub = n->subscribe("/dvrk/" + arm_name + "/position_cartesian_current", 10, &DVRK_MTM::pose_sub_cb, this);
    state_sub = n->subscribe("/dvrk/" + arm_name + "/robot_state", 10, &DVRK_MTM::state_sub_cb, this);
    joint_sub = n->subscribe("/dvrk/" + arm_name + "/position_joint_current", 10, &DVRK_MTM::joint_sub_cb, this);
    clutch_sub = n->subscribe("/dvrk/footpedals/clutch", 10, &DVRK_MTM::clutch_sub_cb, this);

    state_pub = n->advertise<std_msgs::String>("/dvrk/" + arm_name + "/set_robot_state", 10);
    force_pub = n->advertise<geometry_msgs::Wrench>("/dvrk/" + arm_name + "/set_wrench_body", 10);
    force_orientation_safety_pub = n->advertise<std_msgs::Bool>("/dvrk/" + arm_name + "/set_wrench_body_orientation_absolute",10);
    sleep(1);
    //set_mode(std::string("Home"));
    ori_corr.setValue( 0 ,-1 , 0,
                       1 , 0 , 0,
                       0 , 0 ,-1);
    chai_origin[0] = 0;
    chai_origin[1] = 0;
    chai_origin[2] = 0;
    _clutch_pressed = false;
    scale = 0.1;

}

void DVRK_MTM::joint_sub_cb(const sensor_msgs::JointStateConstPtr &msg){
    pre_joint = cur_joint;
    cur_joint = *msg;
}
void DVRK_MTM::pose_sub_cb(const geometry_msgs::PoseStampedConstPtr &msg){
    pre_pose = cur_pose;
    cur_pose = *msg;

    tf::quaternionMsgToTF(cur_pose.pose.orientation, tf_cur_ori);
    mat_ori.setRotation(tf_cur_ori);
    mat_ori =  ori_corr * mat_ori;
}
void DVRK_MTM::state_sub_cb(const std_msgs::StringConstPtr &msg){
    cur_state = *msg;
}

void DVRK_MTM::clutch_sub_cb(const sensor_msgs::JoyConstPtr &msg){
    _clutch_pressed = msg->buttons[0];
}

// TASK::Create an ENUM and check for all the good states
bool DVRK_MTM::_is_mtm_available(){
    if (state_pub.getNumSubscribers() > 0 && state_sub.getNumPublishers() > 0 && pose_sub.getNumPublishers() > 0){
        // If there are listeners to the state_publisher and pose publisher and subscribers to state msg, most likely the MTM is available
        // Doing 3 seperate topic checks for redundancy
        return true;
    }
    else{
        return false;
    }

}

bool DVRK_MTM::_in_effort_mode(){
    if(_is_mtm_available()){
        if(strcmp(cur_state.data.c_str(), _m_effort_mode.c_str()) == 0){
            return true;
        }
    }
    else{
        return false;
    }
}

void DVRK_MTM::get_cur_position(double &x, double &y, double &z){
    if(_clutch_pressed == false){
        chai_origin[0] += scale*(cur_pose.pose.position.y - pre_pose.pose.position.y);
        chai_origin[1] -= scale*(cur_pose.pose.position.x - pre_pose.pose.position.x);
        chai_origin[2] += scale*(cur_pose.pose.position.z - pre_pose.pose.position.z);
    }
    x = chai_origin[0];
    y = chai_origin[1];
    z = chai_origin[2];
}

void DVRK_MTM::_rate_sleep(){
    sleep(0.01);
}

bool DVRK_MTM::set_mode(std::string str){
    state_cmd.data = str;
    state_pub.publish(state_cmd);
    ros::spinOnce();
    rate->sleep();
    if(strcmp(str.c_str(),_m_effort_mode.c_str()) == 0){
        std_msgs::Bool _is_effort_mode;
        _is_effort_mode.data = true;
        force_orientation_safety_pub.publish(_is_effort_mode);
        ros::spinOnce();
        rate->sleep();
        sleep(0.5);
    }
    return _in_effort_mode();
}

bool DVRK_MTM::set_force(double fx, double fy, double fz){
    if(_clutch_pressed == false){
        cmd_wrench.force.x = fx;
        cmd_wrench.force.y = fy;
        cmd_wrench.force.z = fz;
    }
    else{
        cmd_wrench.force.x = 0;
        cmd_wrench.force.y = 0;
        cmd_wrench.force.z = 0;
    }

    force_pub.publish(cmd_wrench);
    ros::spinOnce();
    rate->sleep();
}

DVRK_MTM::~DVRK_MTM(){
    delete n;
    delete rate;
}
