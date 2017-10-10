#include "dvrk_arms/Arm.h"
DVRK_Arm::DVRK_Arm(const std::string &arm_name){
    valid_arms.push_back("MTML");
    valid_arms.push_back("MTMR");
    valid_arms.push_back("PSM1");
    valid_arms.push_back("PSM2");
    valid_arms.push_back("PSM3");

    bool _valid_arm = false;

    for(size_t i = 0; i < valid_arms.size(); i ++){
        if (strcmp(arm_name.c_str(), valid_arms[i].c_str()) == 0){
           _valid_arm = true;
        }
    }

    if(_valid_arm){
        ROS_INFO("Arm %s specified", arm_name.c_str());
        init();
    }
    else{
        ROS_ERROR("%s Invalid Arm Specified", arm_name.c_str());
    }


}

void DVRK_Arm::init(){
    int argc;
    char** argv;
    ros::M_string s;
    ros::init(s, arm_name + "_interface_node");

    n = new ros::NodeHandle;
    rate = new ros::Rate(1000);

    pose_sub = n->subscribe("/dvrk/" + arm_name + "/position_cartesian_current", 10, &DVRK_Arm::pose_sub_cb, this);
    state_sub = n->subscribe("/dvrk/" + arm_name + "/robot_state", 10, &DVRK_Arm::state_sub_cb, this);
    joint_sub = n->subscribe("/dvrk/" + arm_name + "/position_joint_current", 10, &DVRK_Arm::joint_sub_cb, this);
    clutch_sub = n->subscribe("/dvrk/footpedals/clutch", 10, &DVRK_Arm::clutch_sub_cb, this);
    coag_sub = n->subscribe("/dvrk/footpedals/coag", 10, &DVRK_Arm::coag_sub_cb, this);

    joint_pub = n->advertise<sensor_msgs::JointState>("/dvrk/" + arm_name + "/set_position_joint", 10);
    pose_pub  = n->advertise<geometry_msgs::PoseStamped>("/dvrk/" + arm_name + "/set_position_cartesian", 10);
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

void DVRK_Arm::coag_sub_cb(const sensor_msgs::JoyConstPtr &msg){
    _coag_pressed = msg->buttons[0];
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

void DVRK_Arm::get_cur_position(tf::Vector3 &pos){
    pos.setX(cur_pose.pose.position.x);
    pos.setY(cur_pose.pose.position.y);
    pos.setZ(cur_pose.pose.position.z);
}

void DVRK_Arm::get_cur_position(geometry_msgs::Point &pos){
    pos = cur_pose.pose.position;
}

void DVRK_Arm::get_cur_orientation(double &roll, double &pitch, double &yaw){
    mat_ori.getRPY(roll,pitch,yaw);
}

void DVRK_Arm::get_cur_orientation(tf::Quaternion &quat){
    quat = tf_cur_ori;
}

void DVRK_Arm::get_cur_orientation(geometry_msgs::Quaternion &quat){
    quat = gm_cur_ori;
}

void DVRK_Arm::get_cur_orientation(tf::Matrix3x3 &mat){
    mat = mat_ori;
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

bool DVRK_Arm::set_position(const double &x, const double &y, const double &z){
    cmd_pose.pose.position.x = x;
    cmd_pose.pose.position.y = y;
    cmd_pose.pose.position.z = z;
    set_pose(cmd_pose);
}

bool DVRK_Arm::set_position(const geometry_msgs::Point &pos){
    cmd_pose.pose.position = pos;
    set_pose(cmd_pose);
}

bool DVRK_Arm::set_position(const tf::Vector3 &pos){
    cmd_pose.pose.position.x = pos.getX();
    cmd_pose.pose.position.y = pos.getY();
    cmd_pose.pose.position.z = pos.getZ();
    set_pose(cmd_pose);
}

bool DVRK_Arm::set_orientation(const double &roll, const double &pitch, const double &yaw){
    tf::Quaternion tf_quat;
    geometry_msgs::Quaternion gm_quat;
    tf_quat.setRPY(roll, pitch, yaw);
    tf::quaternionTFToMsg(tf_quat, gm_quat);
    cmd_pose.pose.orientation = gm_quat;

    set_pose(cmd_pose);
}

bool DVRK_Arm::set_orientation(const geometry_msgs::Quaternion &quat){
    cmd_pose.pose.orientation = quat;

    set_pose(cmd_pose);
}

bool DVRK_Arm::set_orientation(const tf::Matrix3x3 &mat){
    tf::Quaternion tf_quat;
    geometry_msgs::Quaternion gm_quat;
    mat.getRotation(tf_quat);
    tf::quaternionTFToMsg(tf_quat, gm_quat);
    cmd_pose.pose.orientation = gm_quat;

    set_pose(cmd_pose);
}

bool DVRK_Arm::set_pose(const geometry_msgs::PoseStamped &pose){
    pose_pub.publish(pose);
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
