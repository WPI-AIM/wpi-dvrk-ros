#include "dvrk_chai3d/MTMChai.h"
DVRK_MTM::DVRK_MTM(){
}

void DVRK_MTM::init(){
    int argc;
    char** argv;
    ros::M_string s;
    ros::init(s, "my_node");

    n = new ros::NodeHandle;
    spinner = new ros::AsyncSpinner(1);
    rate = new ros::Rate(1500);

    n->param(std::string("arm"), arm_name, std::string("MTMR"));

    pose_sub = n->subscribe("/dvrk/" + arm_name + "/position_cartesian_current", 10, &DVRK_MTM::pose_sub_cb, this);
    state_sub = n->subscribe("/dvrk/" + arm_name + "/robot_state", 10, &DVRK_MTM::state_sub_cb, this);
    joint_sub = n->subscribe("/dvrk/" + arm_name + "/position_joint_current", 10, &DVRK_MTM::joint_sub_cb, this);

    state_pub = n->advertise<std_msgs::String>("/dvrk/" + arm_name + "/set_robot_state", 10);
    force_pub = n->advertise<geometry_msgs::Wrench>("/dvrk/" + arm_name + "/set_wrench_body", 10);
    force_orientation_safety_pub = n->advertise<std_msgs::Bool>("/dvrk/" + arm_name + "/set_wrench_body_orientation_absolute",10);
    spinner->start();
    sleep(1);
    //set_mode(std::string("Home"));
    ori_corr.setValue( 0 ,-1 , 0,
                       1 , 0 , 0,
                       0 , 0 ,-1);

    pos_offset[0] = 0.18;
    pos_offset[1] = 0.016;
    pos_offset[2] = 0.26;

}

void DVRK_MTM::joint_sub_cb(const sensor_msgs::JointStateConstPtr &msg){
    pre_joint = cur_joint;
    cur_joint = *msg;
}
void DVRK_MTM::pose_sub_cb(const geometry_msgs::PoseStampedConstPtr &msg){
    pre_pose = cur_pose;
    cur_pose = *msg;
    cur_pose.pose.position.x += pos_offset[0];
    cur_pose.pose.position.y += pos_offset[1];
    cur_pose.pose.position.z += pos_offset[2];

    tf::quaternionMsgToTF(cur_pose.pose.orientation, tf_cur_ori);
    mat_ori.setRotation(tf_cur_ori);
    mat_ori =  ori_corr * mat_ori;
    //ROS_INFO("Here %f %f %f", msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
}
void DVRK_MTM::state_sub_cb(const std_msgs::StringConstPtr &msg){
    cur_state = *msg;
}

// TASK::Create an ENUM and check for all the good states
bool DVRK_MTM::_is_mtm_available(){
//    if (strcmp("DVRK_READY", cur_state.data.c_str()) == 0 || strcmp("DVRK_EFFORT_CARTESIAN", cur_state.data.c_str()) == 0){
//        return true;
//    }
//    else
//        return false;
    return true;
}

void DVRK_MTM::_rate_sleep(){
    //rate->sleep();
    sleep(0.01);
}

bool DVRK_MTM::set_mode(std::string str){
    std_msgs::String msg;
    msg.data = str;
    state_pub.publish(msg);
    ros::spinOnce();
    if(strcmp(str.c_str(),_m_effort_mode.c_str()) == 0){
        std_msgs::Bool check;
        check.data = true;
        force_orientation_safety_pub.publish(check);
    }
    //sleep(0.5);
    //ROS_INFO("Setting Robot State to %s", str.c_str());
}

bool DVRK_MTM::set_force(double fx, double fy, double fz){
    cmd_wrench.force.x = fx;
    cmd_wrench.force.y = fy;
    cmd_wrench.force.z = fz;

    force_pub.publish(cmd_wrench);
    ros::spinOnce();
}

DVRK_MTM::~DVRK_MTM(){
    delete n;
    delete spinner;
}
