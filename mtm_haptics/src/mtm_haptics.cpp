#include<iostream>
#include<stdio.h>
#include<ros/ros.h>
#include<sensor_msgs/JointState.h>
#include<geometry_msgs/Pose.h>
#include<std_msgs/String.h>


class MTMHaptics{

public:
    MTMHaptics();

    ros::Publisher crt_torque_pub;
    ros::Publisher jnt_torque_pub;

    ros::Subscriber jnt_pos_sub;
    ros::Subscriber jnt_torque_sub;
    ros::Subscriber crt_pos_sub;
    ros::Subscriber cur_robot_state_sub;




protected:
    ros::NodeHandle node_;
    void jnt_pos_cb(const sensor_msgs::JointStatePtr &msg);
    void jnt_torque_cb(const sensor_msgs::JointStatePtr &msg);
    void crt_pos_cb(const geometry_msgs::PoseConstPtr &msg);
    void cur_robot_state_cb(const std_msgs::StringConstPtr &msg);

    geometry_msgs::Pose cur_mtm_pose;
    std::vector<double> cur_mtm_effort;
    std::vector<double> cur_mtm_jnt_pos;
    std::string cur_robot_state;
};

MTMHaptics::MTMHaptics(){

    jnt_pos_sub = node_.subscribe("/dvrk_mtm/joint_position_current", 10, &MTMHaptics::jnt_pos_cb, this);
    jnt_torque_sub = node_.subscribe("/dvrk_mtm/joint_effort_current", 10, &MTMHaptics::jnt_torque_cb, this);
    crt_pos_sub = node_.subscribe("/dvrk_mtm/cartesian_pose_current", 10, &MTMHaptics::crt_pos_cb, this);

}

void MTMHaptics::jnt_pos_cb(const sensor_msgs::JointStatePtr &msg){
    cur_mtm_jnt_pos = msg->position;
}

void MTMHaptics::jnt_torque_cb(const sensor_msgs::JointStatePtr &msg){
    cur_mtm_effort = msg->effort;
}

void MTMHaptics::crt_pos_cb(const geometry_msgs::PoseConstPtr &msg){
    cur_mtm_pose.position = msg->position;
    cur_mtm_pose.orientation = msg->orientation;

}

void MTMHaptics::cur_robot_state_cb(const std_msgs::StringConstPtr &msg){
    cur_robot_state = msg->data;
}

int main(int argc, char ** argv){

printf("Hello ROS");
return 0;
}
