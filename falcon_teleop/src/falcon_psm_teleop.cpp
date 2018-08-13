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

#include "falcon_teleop/falcon_psm_teleop.h"
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Joy.h"
#include "tf/transform_broadcaster.h"
#include "tf/LinearMath/Transform.h"

geometry_msgs::Pose psm_pose_cur;
sensor_msgs::Joy falcon_pose_cur;
sensor_msgs::Joy falcon_pose_offset;
sensor_msgs::Joy falcon_pose_cmd;
geometry_msgs::Pose psm_pose_cmd;
geometry_msgs::Pose mtm_pose_cur;

bool first = true;
double db = 0.0001;
double precision = 1000;
double scale = 0.2;

std_msgs::String psm_state_cur;

void psm_pose_cb(const geometry_msgs::PoseStampedConstPtr msg){
    psm_pose_cur = msg->pose;
}

void mtm_pose_cb(const geometry_msgs::PoseStampedConstPtr msg){
    mtm_pose_cur = msg->pose;
}

void falcon_pose_cb(const sensor_msgs::JoyConstPtr msg){
    if(first){
        falcon_pose_offset = *msg;
        falcon_pose_cur = *msg;
        first = false;
    }
    else{
    falcon_pose_cur = *msg;
    }
}

void psm_state_cb(const std_msgs::StringConstPtr msg){
    psm_state_cur = *msg;
}

void dead_band(sensor_msgs::Joy &msg){
    for(int i = 0 ; i<msg.axes.size() ; i++){
        double abs_pos = std::abs(msg.axes[i]);
        if(abs_pos < db){
            msg.axes[i] = 0;
        }
        else{
            msg.axes[i] = msg.axes[i] - ((abs_pos/msg.axes[i]) * db);
        }
    }
}

void clip(sensor_msgs::Joy &msg){
    for(int i = 0 ; i<msg.axes.size() ; i++){
        msg.axes[i] = (double)((int)(msg.axes[i] * precision))/precision;
    }
}



int main(int argc, char **argv){

    ros::init(argc, argv, "falcon_teleop");
    ros::NodeHandle node;
    ros::Rate rate(1000);
    tf::TransformBroadcaster t_br;
    tf::Transform trans_falcon, trans_mtm;
    tf::Vector3 pos_falcon, pos_mtm;

    std::string req_state = "DVRK_POSITION_CARTESIAN";

    ros::Subscriber psm_pose_sub;
    ros::Subscriber mtm_pose_sub;
    ros::Subscriber falcon_pose_sub;
    ros::Subscriber psm_state_sub;
    ros::Publisher  psm_teleop;
    ros::Publisher  psm_state_pub;
    ros::Publisher  falcon_joy_pub;
    ros::Publisher  falcon_joy_pub_debug;

    psm_pose_sub = node.subscribe("dvrk/PSM1/position_cartesian_current",10,psm_pose_cb);
    mtm_pose_sub = node.subscribe("dvrk/MTMR/position_cartesian_current",10,mtm_pose_cb);
    psm_state_sub = node.subscribe("dvrk/PSM1/robot_state",10,psm_state_cb);
    falcon_pose_sub = node.subscribe("/falcon/joystick",10,falcon_pose_cb);
    psm_teleop = node.advertise<geometry_msgs::Pose>("/dvrk/PSM1/set_position_cartesian",10);
    psm_state_pub = node.advertise<std_msgs::String>("/dvrk/PSM1/set_robot_state",10);
    falcon_joy_pub = node.advertise<sensor_msgs::Joy>("/falcon/offset_output",10);
    falcon_joy_pub_debug = node.advertise<sensor_msgs::Joy>("/falcon/offset_output_debug",10);

    std_msgs::String state_msg;
    state_msg.data = req_state.c_str();
    ROS_INFO("Setting PSM1 to %s state",req_state.c_str());
    psm_state_pub.publish(state_msg);
    ros::spinOnce();
    rate.sleep();


    falcon_pose_cmd.axes.resize(3);
    falcon_pose_cur.axes.resize(3);
    falcon_pose_offset.axes.resize(3);

    sleep(2);



    int counter = 0;

    while(node.ok()){
        if (!strcmp(psm_state_cur.data.c_str(), req_state.c_str()))
        {
            falcon_pose_cmd.axes[0] = falcon_pose_cur.axes[0] - falcon_pose_offset.axes[0];
            falcon_pose_cmd.axes[1] = falcon_pose_cur.axes[1] - falcon_pose_offset.axes[1];
            falcon_pose_cmd.axes[2] = falcon_pose_cur.axes[2] - falcon_pose_offset.axes[2];

            //falcon_pose_cmd.axes[0] = (falcon_pose_cmd.axes[0]*precision);
            //falcon_pose_cmd.axes[0] = falcon_pose_cmd.axes[0]/precision;
            dead_band(falcon_pose_cmd);
            falcon_joy_pub_debug.publish(falcon_pose_cmd);
            clip(falcon_pose_cmd);

            pos_falcon.setX(falcon_pose_cmd.axes[0]);
            pos_falcon.setY(falcon_pose_cmd.axes[2]);
            pos_falcon.setZ(falcon_pose_cmd.axes[1]);

            pos_mtm.setX(mtm_pose_cur.position.x);
            pos_mtm.setY(mtm_pose_cur.position.y);
            pos_mtm.setZ(mtm_pose_cur.position.z);
            //pos.setW(1);
            //pos.normalize();
            trans_falcon.setOrigin(pos_falcon);
            trans_mtm.setOrigin(pos_mtm);



            psm_pose_cmd.position.x = psm_pose_cur.position.x - (scale * falcon_pose_cmd.axes[0]);
            psm_pose_cmd.position.y = psm_pose_cur.position.y + (scale * falcon_pose_cmd.axes[2]);
            psm_pose_cmd.position.z = psm_pose_cur.position.z + (scale * falcon_pose_cmd.axes[1]);

            if(counter < 20){
            psm_pose_cmd.orientation = psm_pose_cur.orientation;
            }


            psm_teleop.publish(psm_pose_cmd);
            falcon_joy_pub.publish(falcon_pose_cmd);

//            t_br.sendTransform(tf::StampedTransform(trans_falcon,ros::Time::now(),"/world","/falcon_position"));
//            t_br.sendTransform(tf::StampedTransform(trans_mtm,ros::Time::now(),"/world","/mtm_position"));
        }
        else{
            ROS_INFO("DVRK PSM in %s, CURRENT STATE IS %s ", req_state.c_str(), psm_state_cur.data.c_str());
            psm_state_pub.publish(state_msg);
            sleep(2);
        }
        counter++;
        ros::spinOnce();
        rate.sleep();

    }
    return 0;
}
