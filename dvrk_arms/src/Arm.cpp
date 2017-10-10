#include "dvrk_arms/Arm.h"
DVRK_Arm::DVRK_Arm(){
}

void DVRK_Arm::init(){
    int argc;
    char** argv;
    ros::M_string s;
    ros::init(s, "Arm_node");

    n = new ros::NodeHandle;
    rate = new ros::Rate(1000);

    n->param(std::string("arm"), arm_name, std::string("ArmR"));

    pose_sub = n->subscribe("/dvrk/" + arm_name + "/position_cartesian_current", 10, &DVRK_Arm::pose_sub_cb, this);
    state_sub = n->subscribe("/dvrk/" + arm_name + "/robot_state", 10, &DVRK_Arm::state_sub_cb, this);
    joint_sub = n->subscribe("/dvrk/" + arm_name + "/position_joint_current", 10, &DVRK_Arm::joint_sub_cb, this);
    clutch_sub = n->subscribe("/dvrk/footpedals/clutch", 10, &DVRK_Arm::clutch_sub_cb, this);

    state_pub = n->advertise<std_msgs::String>("/dvrk/" + arm_name + "/set_robot_state", 10);
    force_pub = n->advertise<geometry_msgs::Wrench>("/dvrk/" + arm_name + "/set_wrench_body", 10);
    force_orientation_safety_pub = n->advertise<std_msgs::Bool>("/dvrk/" + arm_name + "/set_wrench_body_orientation_absolute",10);
    sleep(1);
    //set_mode(std::string("Home"));
    ori_corr.setValue( 0 ,-1 , 0,
                       1 , 0 , 0,
                       0 , 0 ,-1);
    _clutch_pressed = false;
    scale = 0.1;

}

void DVRK_Arm::joint_sub_cb(const sensor_msgs::JointStateConstPtr &msg){
    pre_joint = cur_joint;
    cur_joint = *msg;
}
void DVRK_Arm::pose_sub_cb(const geometry_msgs::PoseStampedConstPtr &msg){
    pre_pose = cur_pose;
    cur_pose = *msg;

    tf::quaternionMsgToTF(cur_pose.pose.orientation, tf_cur_ori);
    mat_ori.setRotation(tf_cur_ori);
    mat_ori =  ori_corr * mat_ori;
}
void DVRK_Arm::state_sub_cb(const std_msgs::StringConstPtr &msg){
    cur_state = *msg;
}

void DVRK_Arm::clutch_sub_cb(const sensor_msgs::JoyConstPtr &msg){
    _clutch_pressed = msg->buttons[0];
}

// TASK::Create an ENUM and check for all the good states
bool DVRK_Arm::_is_available(){
    if (state_pub.getNumSubscribers() > 0 && state_sub.getNumPublishers() > 0 && pose_sub.getNumPublishers() > 0){
        // If there are listeners to the state_publisher and pose publisher and subscribers to state msg, most likely the Arm is available
        // Doing 3 seperate topic checks for redundancy
        return true;
    }
    else{
        return false;
    }

}

bool DVRK_Arm::_in_effort_mode(){
    if(_is_available()){
        if(strcmp(cur_state.data.c_str(), _m_effort_mode.c_str()) == 0){
            return true;
        }
    }
    else{
        return false;
    }
}

void DVRK_Arm::get_cur_position(double &x, double &y, double &z){
    x = cur_pose.pose.position.x;
    y = cur_pose.pose.position.y;
    z = cur_pose.pose.position.z;
}

void DVRK_Arm::_rate_sleep(){
    sleep(0.01);
}

bool DVRK_Arm::set_mode(std::string str){
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

bool DVRK_Arm::set_force(const double &fx, const double &fy, const double &fz){
    set_wrench(fx, fy, fz, 0 , 0 , 0);
}

bool DVRK_Arm::set_moment(const double &nx, const double &ny, const double &nz){
    set_wrench(0, 0, 0, nx, ny, nz);
}

bool DVRK_Arm::set_wrench(const double &fx,const double &fy,const double &fz,const double &nx,const double &ny,const double &nz){
    if(_clutch_pressed == false){
        cmd_wrench.force.x = fx;
        cmd_wrench.force.y = fy;
        cmd_wrench.force.z = fz;

        cmd_wrench.torque.x = nx;
        cmd_wrench.torque.y = ny;
        cmd_wrench.torque.z = nz;
    }
    else{
        cmd_wrench.force.x = 0;
        cmd_wrench.force.y = 0;
        cmd_wrench.force.z = 0;

        cmd_wrench.torque.x = 0;
        cmd_wrench.torque.y = 0;
        cmd_wrench.torque.z = 0;
    }

    force_pub.publish(cmd_wrench);
    ros::spinOnce();
    rate->sleep();
}

DVRK_Arm::~DVRK_Arm(){
    delete n;
    delete rate;
}
