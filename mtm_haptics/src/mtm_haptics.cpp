#include<iostream>
#include<stdio.h>
#include<ros/ros.h>
#include<sensor_msgs/JointState.h>
#include<geometry_msgs/Pose.h>


class MTMHaptics{

public:
    MTMHaptics();

    ros::Publisher crt_torque_pub;
    ros::Publisher jnt_torque_pub;

    ros::Subscriber jnt_pos_sub;
    ros::Subscriber jnt_torque_sub;




protected:
    ros::NodeHandle node_;
    void jnt_pos_cb(const sensor_msgs::JointState &msg);
    void jnt_torque_cb(const sensor_msgs::JointState &msg);
};

MTMHaptics::MTMHaptics(){
    jnt_pos_sub = node_.subscribe("/dvrk_mtm/joint_position_current", 10, &MTMHaptics::jnt_pos_cb, this);
    jnt_torque_sub = node_.subscribe("/dvrk_mtm/joint_effort_current", 10, &MTMHaptics::jnt_torque_cb, this);
}

void MTMHaptics::jnt_pos_cb(const sensor_msgs::JointState &msg){

}

void MTMHaptics::jnt_torque_cb(const sensor_msgs::JointState &msg){

}

int main(int argc, char ** argv){

printf("Hello ROS");
return 0;
}
