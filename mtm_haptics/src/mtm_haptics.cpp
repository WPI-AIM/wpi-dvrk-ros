#include<iostream>
#include<stdio.h>
#include<ros/ros.h>
#include<sensor_msgs/JointState.h>
#include<geometry_msgs/Pose.h>
#include<geometry_msgs/Vector3.h>
#include<geometry_msgs/Wrench.h>
#include<std_msgs/String.h>

#include <mtm_haptics/haptic_position_kinematics.h>


class MTMHaptics{

public:
    MTMHaptics();

    ros::Publisher crt_torque_pub;
    ros::Publisher jnt_torque_pub;
    ros::Publisher robot_state_pub;

    ros::Subscriber jnt_pos_sub;
    ros::Subscriber jnt_torque_sub;
    ros::Subscriber crt_pos_sub;
    ros::Subscriber cur_robot_state_sub;
    void haptic_feeback_plane(geometry_msgs::Point set_point, bool _collision);
    void set_effort_mode();
    ros::Rate *rate_;




protected:
    ros::NodeHandle node_;
    void jnt_pos_cb(const sensor_msgs::JointStatePtr &msg);
    void jnt_torque_cb(const sensor_msgs::JointStatePtr &msg);
    void crt_pos_cb(const geometry_msgs::PoseConstPtr &msg);
    void cur_robot_state_cb(const std_msgs::StringConstPtr &msg);

    geometry_msgs::Pose cur_mtm_pose;
    geometry_msgs::Pose pre_mtm_pose;
    geometry_msgs::Vector3 cur_mtm_tip_vel;
    std::vector<double> cur_mtm_effort;
    std::vector<double> cur_mtm_jnt_pos;
    std::string cur_robot_state;
    std_msgs::String robot_state_cmd;
    geometry_msgs::Wrench haptic_feedback;
    geometry_msgs::Vector3 Kp;
    geometry_msgs::Vector3 Kd;
    geometry_msgs::Vector3 dX;
    sensor_msgs::JointState torque_msg;


};

MTMHaptics::MTMHaptics(){
    jnt_pos_sub = node_.subscribe("/dvrk_mtm/joint_position_current", 10, &MTMHaptics::jnt_pos_cb, this);
//    jnt_torque_sub = node_.subscribe("/dvrk_mtm/joint_effort_current", 10, &MTMHaptics::jnt_torque_cb, this);
    crt_pos_sub = node_.subscribe("/dvrk_mtm/cartesian_pose_current", 10, &MTMHaptics::crt_pos_cb, this);

    crt_torque_pub = node_.advertise<geometry_msgs::Wrench>("/dvrk_mtm/set_wrench_body",10);
    jnt_torque_pub = node_.advertise<sensor_msgs::JointState>("/dvrk_mtm/set_effort_joint_unsinked",10);
    robot_state_pub = node_.advertise<std_msgs::String>("/dvrk_mtm/set_robot_state",10);

    rate_ = new ros::Rate(100);

    Kp.x = 50;
    Kp.y = 50;
    Kp.z = 50;

    Kd.x = 10;
    Kd.y = 10;
    Kd.z = 10;

    dX.x = 0.025;
    dX.y = 0.025;
    dX.z = 0.025;

}

void MTMHaptics::jnt_pos_cb(const sensor_msgs::JointStatePtr &msg){
    cur_mtm_jnt_pos = msg->position;
}

void MTMHaptics::jnt_torque_cb(const sensor_msgs::JointStatePtr &msg){
    cur_mtm_effort = msg->effort;
}

void MTMHaptics::crt_pos_cb(const geometry_msgs::PoseConstPtr &msg){
    pre_mtm_pose.position = cur_mtm_pose.position;
    pre_mtm_pose.orientation = cur_mtm_pose.orientation;
    cur_mtm_pose.position = msg->position;
    cur_mtm_pose.orientation = msg->orientation;

    cur_mtm_tip_vel.x = cur_mtm_pose.position.x - pre_mtm_pose.position.x;
    cur_mtm_tip_vel.y = cur_mtm_pose.position.y - pre_mtm_pose.position.y;
    cur_mtm_tip_vel.z = cur_mtm_pose.position.z - pre_mtm_pose.position.z;
}

void MTMHaptics::cur_robot_state_cb(const std_msgs::StringConstPtr &msg){
    cur_robot_state = msg->data;
}

void MTMHaptics::set_effort_mode(){
    robot_state_cmd.data = "DVRK_EFFORT_CARTESIAN";
    robot_state_pub.publish(robot_state_cmd);
    ROS_INFO("Setting Robot state as: %s",robot_state_cmd.data.c_str());
    rate_->sleep();
}

void MTMHaptics::haptic_feeback_plane(geometry_msgs::Point set_point, bool _collision){
    if(strcmp(this->cur_robot_state.c_str(),"DVRK_EFFORT_CARTESIAN")){
        geometry_msgs::Point error;
        error.x = set_point.x - cur_mtm_pose.position.x;
        error.y = set_point.y - cur_mtm_pose.position.y;
        error.z = set_point.z - cur_mtm_pose.position.z;

        if (error.y >= 0 && error.y <= dX.y){
            haptic_feedback.force.x = (Kp.x * error.y) - (Kd.y *cur_mtm_tip_vel.y);
            haptic_feedback.force.y = 0; //Currently applying a force in the Z direction, as Z for tip point towards Y of base, and wrench is being applied in tip frame.
            haptic_feedback.force.z = 0;
            ROS_INFO("Publishing Force");
            //compute_torques(haptic_feedback,torque_msg);
            //jnt_torque_pub.publish(torque_msg);
            crt_torque_pub.publish(haptic_feedback);
        }
        else{
            haptic_feedback.force.x = 0;
            haptic_feedback.force.y = 0;
            haptic_feedback.force.z = 0;
            crt_torque_pub.publish(haptic_feedback);
        }

    }
    else{
        ROS_WARN("MTM is %s state, it needs to be in DVRK_EFFORT_CARTESIAN STATE",this->cur_robot_state.c_str());
    }
}



int main(int argc, char ** argv){
    ros::init(argc,argv,"mtm_haptics_node");
    MTMHaptics haptics;
    MTM_pos_kinematics test;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ROS_INFO("first check");
    sleep(2.0);
    ROS_INFO("second check");
    geometry_msgs::Point haptic_point;
    haptic_point.y = -0.38;
    haptics.set_effort_mode();

    while (ros::ok()){
    haptics.haptic_feeback_plane(haptic_point,true);
    haptics.rate_->sleep();
    }
}
