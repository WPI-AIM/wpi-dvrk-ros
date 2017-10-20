#include "dvrk_arm/Bridge.h"


DVRK_Bridge::DVRK_Bridge(const std::string &arm_name){
    valid_arms.push_back("MTML");
    valid_arms.push_back("MTMR");
    valid_arms.push_back("PSM1");
    valid_arms.push_back("PSM2");
    valid_arms.push_back("PSM3");

    bool _valid_arm = false;

    for(size_t i = 0; i < valid_arms.size(); i ++){
        if (strcmp(arm_name.c_str(), valid_arms[i].c_str()) == 0){
            this->arm_name = valid_arms[i];
           _valid_arm = true;
        }
    }

    if(_valid_arm){
        ROS_INFO("Specified arm is %s:", arm_name.c_str());
        init();
    }
    else{
        ROS_ERROR("%s Invalid Arm Specified", arm_name.c_str());
    }
}

void DVRK_Bridge::init(){
    int argc;
    char** argv;
    ros::M_string s;
    ros::init(s, arm_name + "_interface_node");

    n = new ros::NodeHandle;
    rate = new ros::Rate(1000);

    pose_sub = n->subscribe("/dvrk/" + arm_name + "/position_cartesian_current", 10, &DVRK_Bridge::pose_sub_cb, this);
    state_sub = n->subscribe("/dvrk/" + arm_name + "/robot_state", 10, &DVRK_Bridge::state_sub_cb, this);
    joint_sub = n->subscribe("/dvrk/" + arm_name + "/position_joint_current", 10, &DVRK_Bridge::joint_sub_cb, this);

    joint_pub = n->advertise<sensor_msgs::JointState>("/dvrk/" + arm_name + "/set_position_joint", 10);
    pose_pub  = n->advertise<geometry_msgs::Pose>("/dvrk/" + arm_name + "/set_position_cartesian", 10);
    state_pub = n->advertise<std_msgs::String>("/dvrk/" + arm_name + "/set_robot_state", 10);
    force_pub = n->advertise<geometry_msgs::Wrench>("/dvrk/" + arm_name + "/set_wrench_body", 10);
    force_orientation_safety_pub = n->advertise<std_msgs::Bool>("/dvrk/" + arm_name + "/set_wrench_body_orientation_absolute",10);

    DVRK_FootPedals::init(n);
    sleep(1);
    scale = 0.1;
    ros::spinOnce();
    rate->sleep();
}

void DVRK_Bridge::joint_sub_cb(const sensor_msgs::JointStateConstPtr &msg){
    pre_joint = cur_joint;
    cur_joint = *msg;
}
void DVRK_Bridge::pose_sub_cb(const geometry_msgs::PoseStampedConstPtr &msg){
    pre_pose = cur_pose;
    cur_pose = *msg;
}

void DVRK_Bridge::state_sub_cb(const std_msgs::StringConstPtr &msg){
    cur_state = *msg;
}

void DVRK_Bridge::_rate_sleep(){
    ros::spinOnce();
    rate->sleep();
}

DVRK_Bridge::~DVRK_Bridge(){
    delete n;
    delete rate;
}


