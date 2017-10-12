#include "dvrk_arm/Arm.h"
DVRK_Arm::DVRK_Arm(const std::string &arm_name){
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
    pose_pub  = n->advertise<geometry_msgs::Pose>("/dvrk/" + arm_name + "/set_position_cartesian", 10);
    state_pub = n->advertise<std_msgs::String>("/dvrk/" + arm_name + "/set_robot_state", 10);
    force_pub = n->advertise<geometry_msgs::Wrench>("/dvrk/" + arm_name + "/set_wrench_body", 10);
    force_orientation_safety_pub = n->advertise<std_msgs::Bool>("/dvrk/" + arm_name + "/set_wrench_body_orientation_absolute",10);
    sleep(1);
    //set_mode(std::string("Home"));
    origin_pos.setX(0);
    origin_pos.setY(0);
    origin_pos.setZ(0);
    reorient_mat.setIdentity();
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

    cisstPose_to_userTransform(cur_pose);
}

void DVRK_Arm::cisstPose_to_userTransform(const geometry_msgs::PoseStamped &pose){

    cisst_pos.setX(pose.pose.position.x);
    cisst_pos.setY(pose.pose.position.y);
    cisst_pos.setZ(pose.pose.position.z);
    tf::quaternionMsgToTF(pose.pose.orientation, cisst_ori_quat);

    cisst_trans.setOrigin(cisst_pos);
    cisst_trans.setRotation(cisst_ori_quat);
    ee_trans = origin_trans * cisst_trans;

    ee_pos = ee_trans.getOrigin();
    ee_ori_quat = ee_trans.getRotation();
    ee_ori_mat.setRotation(ee_ori_quat);
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

bool DVRK_Arm::_in_cart_pos_mode(){
    if(_is_available()){
        if(strcmp(cur_state.data.c_str(), _m_cart_pos_mode.c_str()) == 0){
            return true;
        }
    }
    else{
        return false;
    }
}

bool DVRK_Arm::_in_jnt_pos_mode(){
    if(_is_available()){
        if(strcmp(cur_state.data.c_str(), _m_jnt_pos_mode.c_str()) == 0){
            return true;
        }
    }
    else{
        return false;
    }
}

void DVRK_Arm::set_origin_trans(const tf::Vector3 &pos, const tf::Matrix3x3 &tf_mat){
    tf_mat.getRotation(origin_ori_quat);
    set_origin_trans(pos, origin_ori_quat);
}

void DVRK_Arm::set_origin_trans(const tf::Vector3 &pos, const tf::Quaternion &tf_quat){
    origin_trans.setOrigin(pos);
    origin_trans.setRotation(tf_quat);
}

void DVRK_Arm::set_origin_pos(const double &x, const double &y, const double &z){
    origin_pos.setX(x);
    origin_pos.setY(y);
    origin_pos.setZ(z);

    set_origin_trans(origin_pos, origin_ori_quat);
}

void DVRK_Arm::set_origin_pos(const geometry_msgs::Point &pos){
    origin_pos.setX(pos.x);
    origin_pos.setY(pos.y);
    origin_pos.setZ(pos.z);

    set_origin_trans(origin_pos, origin_ori_quat);
}

void DVRK_Arm::set_origin_pos(const tf::Vector3 &pos){
    origin_pos = pos;

    set_origin_trans(origin_pos, origin_ori_quat);
}

void DVRK_Arm::set_origin_rot(const double &roll, const double &pitch, const double &yaw){
    origin_ori_quat.setRPY(roll, pitch, yaw);

    set_origin_trans(origin_pos, origin_ori_quat);
}

void DVRK_Arm::set_origin_rot(const tf::Quaternion &tf_quat){
    origin_ori_quat = tf_quat;

    set_origin_trans(origin_pos, origin_ori_quat);
}

void DVRK_Arm::set_origin_rot(const geometry_msgs::Quaternion &gm_quat){
    tf::quaternionMsgToTF(gm_quat, origin_ori_quat);

    set_origin_trans(origin_pos, origin_ori_quat);
}

void DVRK_Arm::set_origin_rot(const tf::Matrix3x3 &mat){
    mat.getRotation(origin_ori_quat);

    set_origin_trans(origin_pos, origin_ori_quat);
}

void DVRK_Arm::get_cur_position(double &x, double &y, double &z){
    x = ee_pos.getX();
    y = ee_pos.getY();
    z = ee_pos.getZ();
}

void DVRK_Arm::get_cur_position(tf::Vector3 &pos){
    pos = ee_pos;
}

void DVRK_Arm::get_cur_position(geometry_msgs::Point &pos){
    pos.x = ee_pos.getX();
    pos.y = ee_pos.getY();
    pos.z = ee_pos.getZ();
}

void DVRK_Arm::get_cur_orientation(double &roll, double &pitch, double &yaw){
    ee_ori_mat.getRPY(roll, pitch, yaw);
}

void DVRK_Arm::get_cur_orientation(tf::Quaternion &tf_quat){
    tf_quat = ee_ori_quat;
}

void DVRK_Arm::get_cur_orientation(geometry_msgs::Quaternion &gm_quat){
    tf::quaternionTFToMsg(ee_ori_quat, gm_quat);
}

void DVRK_Arm::get_cur_orientation(tf::Matrix3x3 &mat){
    mat.setRotation(ee_ori_quat);
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
    tf_quat.setRPY(roll, pitch, yaw);
    tf::quaternionTFToMsg(tf_quat, cmd_pose.pose.orientation);

    set_pose(cmd_pose);
}

bool DVRK_Arm::set_orientation(const tf::Quaternion &tf_quat){
    tf::quaternionTFToMsg(tf_quat, cmd_pose.pose.orientation);

    set_pose(cmd_pose);
}

bool DVRK_Arm::set_orientation(const geometry_msgs::Quaternion &gm_quat){
    cmd_pose.pose.orientation = gm_quat;

    set_pose(cmd_pose);
}

bool DVRK_Arm::set_orientation(const tf::Matrix3x3 &mat){
    tf::Quaternion tf_quat;
    mat.getRotation(tf_quat);
    tf::quaternionTFToMsg(tf_quat, cmd_pose.pose.orientation);

    set_pose(cmd_pose);
}

void DVRK_Arm::userPose_to_cisstPose(geometry_msgs::PoseStamped &pose){
    cisst_pos.setX(pose.pose.position.x);
    cisst_pos.setY(pose.pose.position.y);
    cisst_pos.setZ(pose.pose.position.z);

    tf::quaternionMsgToTF(pose.pose.orientation, cisst_ori_quat);
    cisst_trans.setOrigin(cisst_pos);
    cisst_trans.setRotation(cisst_ori_quat);

    cisst_trans = origin_trans.inverse() * cisst_trans;
    pose.pose.position.x = cisst_trans.getOrigin().getX();
    pose.pose.position.y = cisst_trans.getOrigin().getY();
    pose.pose.position.z = cisst_trans.getOrigin().getZ();

    tf::quaternionTFToMsg(cisst_trans.getRotation(), pose.pose.orientation);
}

bool DVRK_Arm::set_pose(geometry_msgs::PoseStamped &pose){
    userPose_to_cisstPose(pose);
    pose_pub.publish(pose.pose);
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
