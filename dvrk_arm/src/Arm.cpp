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

    ee_pos.setX(pose.pose.position.x);
    ee_pos.setY(pose.pose.position.y);
    ee_pos.setZ(pose.pose.position.z);
    tf::quaternionMsgToTF(pose.pose.orientation, ee_ori_quat);

    ee_trans.setOrigin(ee_pos);
    ee_trans.setRotation(ee_ori_quat);
    ee_trans = origin_trans * ee_trans;

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
    x = ee_trans.getOrigin().getX();
    y = ee_trans.getOrigin().getY();
    z = ee_trans.getOrigin().getZ();
}

void DVRK_Arm::get_cur_position(tf::Vector3 &pos){
    pos = ee_trans.getOrigin();
}

void DVRK_Arm::get_cur_position(geometry_msgs::Point &pos){
    tf::pointTFToMsg(ee_trans.getOrigin(), pos);
}

void DVRK_Arm::get_cur_orientation(double &roll, double &pitch, double &yaw){
    tf::Matrix3x3(ee_trans.getRotation()).getRPY(roll, pitch, yaw);
}

void DVRK_Arm::get_cur_orientation(tf::Quaternion &tf_quat){
    tf_quat = ee_trans.getRotation();
}

void DVRK_Arm::get_cur_orientation(geometry_msgs::Quaternion &gm_quat){
    tf::quaternionTFToMsg(ee_trans.getRotation(), gm_quat);
}

void DVRK_Arm::get_cur_orientation(tf::Matrix3x3 &mat){
    mat.setRotation(ee_trans.getRotation());
}

void DVRK_Arm::get_cur_transform(tf::Transform &trans){
    trans = ee_trans;
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
    ee_trans_cmd.setOrigin(tf::Vector3(x,y,z));
    move_arm_cartesian(ee_trans_cmd);
}

bool DVRK_Arm::set_position(const geometry_msgs::Point &pos){
    ee_trans_cmd.setOrigin(tf::Vector3(pos.x, pos.y, pos.z));
    move_arm_cartesian(ee_trans_cmd);
}

bool DVRK_Arm::set_position(const tf::Vector3 &pos){
    ee_trans_cmd.setOrigin(pos);
    move_arm_cartesian(ee_trans_cmd);
}

bool DVRK_Arm::set_orientation(const double &roll, const double &pitch, const double &yaw){
    ee_ori_quat_cmd.setRPY(roll, pitch, yaw);
    ee_trans_cmd.setRotation(ee_ori_quat);
    move_arm_cartesian(ee_trans_cmd);
}

bool DVRK_Arm::set_orientation(const tf::Quaternion &tf_quat){
    ee_trans_cmd.setRotation(tf_quat);
    move_arm_cartesian(ee_trans_cmd);
}

bool DVRK_Arm::set_orientation(const geometry_msgs::Quaternion &gm_quat){
    ee_trans_cmd.setRotation(tf::Quaternion(gm_quat.x, gm_quat.y, gm_quat.z, gm_quat.w));
    move_arm_cartesian(ee_trans_cmd);
}

bool DVRK_Arm::set_orientation(const tf::Matrix3x3 &mat){
    mat.getRotation(ee_ori_quat_cmd);
    ee_trans_cmd.setRotation(ee_ori_quat_cmd);
    move_arm_cartesian(ee_trans_cmd);
}

bool DVRK_Arm::set_pose(geometry_msgs::PoseStamped &pose){
    ee_trans_cmd.setOrigin(tf::Vector3(pose.pose.position.x,
                           pose.pose.position.y,
                           pose.pose.position.z));

    ee_trans_cmd.setRotation(tf::Quaternion(pose.pose.orientation.x,
                             pose.pose.orientation.y,
                             pose.pose.orientation.z,
                             pose.pose.orientation.w));
    move_arm_cartesian(ee_trans_cmd);
}

bool DVRK_Arm::set_transform(tf::Transform &trans){
    ee_trans_cmd = trans;
    move_arm_cartesian(ee_trans_cmd);
}

void DVRK_Arm::move_arm_cartesian(tf::Transform &trans){
    trans = origin_trans.inverse() * trans;
    cmd_pose.pose.position.x = trans.getOrigin().getX();
    cmd_pose.pose.position.y = trans.getOrigin().getY();
    cmd_pose.pose.position.z = trans.getOrigin().getZ();
    tf::quaternionTFToMsg(trans.getRotation(), cmd_pose.pose.orientation);

    pose_pub.publish(cmd_pose);
}

bool DVRK_Arm::set_force(const double &fx, const double &fy, const double &fz){
    ee_force_cmd.setX(fx);
    ee_force_cmd.setY(fy);
    ee_force_cmd.setZ(fz);

    set_arm_wrench(ee_force_cmd, ee_moment_cmd);
}

bool DVRK_Arm::set_moment(const double &nx, const double &ny, const double &nz){
    ee_moment_cmd.setX(nx);
    ee_moment_cmd.setX(ny);
    ee_moment_cmd.setX(nz);

    set_arm_wrench(ee_force_cmd, ee_moment_cmd);
}

bool DVRK_Arm::set_wrench(const double &fx,const double &fy,const double &fz,const double &nx,const double &ny,const double &nz){

    ee_force_cmd.setX(fx);
    ee_force_cmd.setY(fy);
    ee_force_cmd.setZ(fz);

    ee_moment_cmd.setX(nx);
    ee_moment_cmd.setX(ny);
    ee_moment_cmd.setX(nz);

    set_arm_wrench(ee_force_cmd, ee_moment_cmd);
}

void DVRK_Arm::set_arm_wrench(tf::Vector3 &force, tf::Vector3 &moment){
    if(_clutch_pressed == true){
        force.setZero();
        moment.setZero();
    }
    tf::vector3TFToMsg(origin_trans.inverse() * force, cmd_wrench.force);
    tf::vector3TFToMsg(origin_trans.inverse() * moment, cmd_wrench.torque);
    force_pub.publish(cmd_wrench);
    ros::spinOnce();
    rate->sleep();

}

DVRK_Arm::~DVRK_Arm(){
    delete n;
    delete rate;
}
