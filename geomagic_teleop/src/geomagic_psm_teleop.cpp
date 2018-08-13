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

#include "geomagic_teleop/geomagic_psm_teleop.h"
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geomagic_teleop/geomagic_psm_teleop.h"

Geomagic_Teleop::Geomagic_Teleop(boost::shared_ptr<ros::NodeHandle> node):
    _node(node), _clutch(false), _coag(false), _first_trigger(false)
{
    _node->param(std::string("device_name"), _device_name, std::string("Geomagic"));
    _node->param(std::string("arm"), _slave_name, std::string("PSM1"));
    _node->param(std::string("rate"), _pub_rate, 1000);

    rate.reset(new ros::Rate(_pub_rate));
    arm_psm.reset(new DVRK_Arm(_slave_name));
    cur_gFrame.reset(new Frame);
    pre_gFrame.reset(new Frame);

    geomagic_joy_sub = _node->subscribe("/" + _device_name + "/joy",10,&Geomagic_Teleop::geomagic_joy_cb, this);
    geomagic_pose_sub = _node->subscribe("/" + _device_name + "/pose",10,&Geomagic_Teleop::geomagic_pose_cb, this);
    geomagic_force_pub = _node->advertise<geomagic_control::DeviceFeedback>("/" + _device_name + "/force_feedback",10);

    geomagic_joy_cmd.axes.resize(6);
    geomagic_joy_cur.axes.resize(6);
    geomagic_joy_pre.axes.resize(6);
    geomagic_joy_cur.buttons.resize(2);
    device_feedback.lock.resize(3);
    ros::spinOnce();
    arm_psm->set_mode(arm_psm->_m_cart_pos_mode);
    tf::Quaternion temp_quat;
    temp_quat.setRPY(0, 0, M_PI);
    arm_psm->set_origin_frame_rot(temp_quat.inverse());
    sleep(1);
    scale = 0.01;
    align_end_effectors();
}

void Geomagic_Teleop::align_end_effectors(){
    ros::spinOnce();
    tf::Quaternion rot_psm;
    arm_psm->measured_cp_ori(rot_psm);
    rotPsm2Geo = cur_gFrame->rot_quat.inverse() * rot_psm;
    rotGeoLast = cur_gFrame->rot_quat;
    arm_psm->measured_cp_ori(rotPsmLast);
}

void Geomagic_Teleop::PosetoFrame(const geometry_msgs::Pose &pose, FramePtr frame){
    frame->trans.setOrigin(tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
    frame->trans.setRotation(tf::Quaternion(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w));
    frame->pos = frame->trans.getOrigin();
    frame->rot_quat = frame->trans.getRotation();
    frame->rot_mat.setRotation(frame->rot_quat);
}

void Geomagic_Teleop::geomagic_pose_cb(const geometry_msgs::PoseStampedConstPtr msg){
    PosetoFrame(geomagic_pose_cur, pre_gFrame);
    geomagic_pose_pre = geomagic_pose_cur;
    geomagic_pose_cur = msg->pose;
    PosetoFrame(geomagic_pose_cur, cur_gFrame);

}

void Geomagic_Teleop::geomagic_joy_cb(const sensor_msgs::JoyConstPtr msg){
        geomagic_joy_pre = geomagic_joy_cur;
        geomagic_joy_cur = *msg;
        _coag = geomagic_joy_cur.buttons[0];
        _clutch = geomagic_joy_cur.buttons[1];
}


void Geomagic_Teleop::dead_band(sensor_msgs::Joy &msg){
    for(unsigned int i = 0 ; i<msg.axes.size() ; i++){
        double abs_pos = std::abs(msg.axes[i]);
        if(abs_pos < db){
            msg.axes[i] = 0;
        }
        else{
            msg.axes[i] = msg.axes[i] - ((abs_pos/msg.axes[i]) * db);
        }
    }
}

void Geomagic_Teleop::clip(sensor_msgs::Joy &msg){
    for(unsigned int i = 0 ; i<msg.axes.size() ; i++){
        msg.axes[i] = (double)((int)(msg.axes[i] * precision))/precision;
    }
}

void Geomagic_Teleop::run(){
    tf::Transform psmTrans;
    arm_psm->measured_cp(psmTrans);
    psmTrans.setOrigin(psmTrans.getOrigin() + scale*(cur_gFrame->trans.getOrigin() - pre_gFrame->trans.getOrigin()));
    psmTrans.setRotation(rotPsmLast * rotGeoLast.inverse() * cur_gFrame->rot_quat);

    if(_clutch || _coag){     
        device_feedback.position.x = geomagic_pose_cur.position.x;
        device_feedback.position.y = geomagic_pose_cur.position.y;
        device_feedback.position.z = geomagic_pose_cur.position.z;

        device_feedback.force.x = 0; device_feedback.force.y = 0; device_feedback.force.z = 0;

        for(u_int i = 0 ; i < 3 ; i++){
            device_feedback.lock[i] = true;
        }
        if(_coag){
            arm_psm->move_cp(psmTrans);
        }
    }
    else{
        align_end_effectors();
    }

    geomagic_force_pub.publish(device_feedback);
    ros::spinOnce();
    rate->sleep();
}

Geomagic_Teleop::~Geomagic_Teleop(){

}


int main(int argc, char **argv){

    ros::init(argc, argv, "geomagic_teleop");
    boost::shared_ptr<ros::NodeHandle> node(new ros::NodeHandle);
    Geomagic_Teleop geo_teleop(node);

    while(node->ok()){
        geo_teleop.run();
    }
    return 0;
}
