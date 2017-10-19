#include "dvrk_arm/Arm.h"
DVRK_Arm::DVRK_Arm(const std::string &arm_name): DVRK_Bridge(arm_name){
}

void DVRK_Arm::init(){
    origin_trans.setOrigin(tf::Vector3(0,0,0));
    tf::Quaternion temp_quat;
    temp_quat.setRPY(0,0,0);
    origin_trans.setRotation(temp_quat);
    tip_trans.setOrigin(tf::Vector3(0,0,0));
    tip_trans.setRotation(temp_quat);
    ee_trans_cmd.setRotation(temp_quat);
}

void DVRK_Arm::cisstPose_to_userTransform(const geometry_msgs::PoseStamped &pose){

    ee_pos.setX(pose.pose.position.x);
    ee_pos.setY(pose.pose.position.y);
    ee_pos.setZ(pose.pose.position.z);
    tf::quaternionMsgToTF(pose.pose.orientation, ee_ori_quat);

    ee_trans.setOrigin(ee_pos);
    ee_trans.setRotation(ee_ori_quat);
    ee_trans = origin_trans * ee_trans * tip_trans;

    ee_pos = ee_trans.getOrigin();
    ee_ori_quat = ee_trans.getRotation();
    ee_ori_mat.setRotation(ee_ori_quat);
}

void DVRK_Arm::set_origin_frame(const tf::Vector3 &pos, const tf::Matrix3x3 &tf_mat){
    _rate_sleep();
    tf_mat.getRotation(origin_ori_quat);
    set_origin_frame(pos, origin_ori_quat);
}

void DVRK_Arm::set_origin_frame(const tf::Vector3 &pos, const tf::Quaternion &tf_quat){
    _rate_sleep();
    origin_trans.setOrigin(pos);
    origin_trans.setRotation(tf_quat);
}

void DVRK_Arm::set_origin_frame(const tf::Transform &trans){
    _rate_sleep();
    origin_trans = trans;
}

void DVRK_Arm::set_origin_frame_pos(const double &x, const double &y, const double &z){
    _rate_sleep();
    origin_pos.setX(x);
    origin_pos.setY(y);
    origin_pos.setZ(z);

    set_origin_frame(origin_pos, origin_ori_quat);
}

void DVRK_Arm::set_origin_frame_pos(const geometry_msgs::Point &pos){
    _rate_sleep();
    origin_pos.setX(pos.x);
    origin_pos.setY(pos.y);
    origin_pos.setZ(pos.z);

    set_origin_frame(origin_pos, origin_ori_quat);
}

void DVRK_Arm::set_origin_frame_pos(const tf::Vector3 &pos){
    _rate_sleep();
    origin_pos = pos;

    set_origin_frame(origin_pos, origin_ori_quat);
}

void DVRK_Arm::set_origin_frame_rot(const double &roll, const double &pitch, const double &yaw){
    _rate_sleep();
    origin_ori_quat.setRPY(roll, pitch, yaw);

    set_origin_frame(origin_pos, origin_ori_quat);
}

void DVRK_Arm::set_origin_frame_rot(const tf::Quaternion &tf_quat){
    _rate_sleep();
    origin_ori_quat = tf_quat;

    set_origin_frame(origin_pos, origin_ori_quat);
}

void DVRK_Arm::set_origin_frame_rot(const geometry_msgs::Quaternion &gm_quat){
    _rate_sleep();
    tf::quaternionMsgToTF(gm_quat, origin_ori_quat);

    set_origin_frame(origin_pos, origin_ori_quat);
}

void DVRK_Arm::set_origin_frame_rot(const tf::Matrix3x3 &mat){
    _rate_sleep();
    mat.getRotation(origin_ori_quat);

    set_origin_frame(origin_pos, origin_ori_quat);
}

void DVRK_Arm::affix_tip_frame_pos(const double &x, const double &y, const double &z){
    _rate_sleep();
    tip_pos.setX(x);
    tip_pos.setY(y);
    tip_pos.setZ(z);
    affix_tip_frame(tip_pos, tip_ori_quat);
}

void DVRK_Arm::affix_tip_frame_pos(const geometry_msgs::Point &pos){
    _rate_sleep();
    tip_pos.setX(pos.x);
    tip_pos.setY(pos.y);
    tip_pos.setZ(pos.z);
    affix_tip_frame(tip_pos, tip_ori_quat);
}

void DVRK_Arm::affix_tip_frame_pos(const tf::Vector3 &pos){
    _rate_sleep();
    tip_pos = pos;
    affix_tip_frame(tip_pos, tip_ori_quat);
}

void DVRK_Arm::affix_tip_frame_rot(const double &roll, const double &pitch, const double &yaw){
    _rate_sleep();
    tip_ori_quat.setRPY(roll, pitch, yaw);
    affix_tip_frame(tip_pos, tip_ori_quat);
}

void DVRK_Arm::affix_tip_frame_rot(const double &quat_x, const double &quat_y, const double &quat_z, const double &quat_w){
    _rate_sleep();
    tip_ori_quat.setX(quat_x);
    tip_ori_quat.setY(quat_y);
    tip_ori_quat.setZ(quat_z);
    tip_ori_quat.setW(quat_w);
    affix_tip_frame(tip_pos, tip_ori_quat);
}

void DVRK_Arm::affix_tip_frame_rot(const geometry_msgs::Quaternion &gm_quat){
    _rate_sleep();
    tf::quaternionMsgToTF(gm_quat, tip_ori_quat);
    affix_tip_frame(tip_pos, tip_ori_quat);
}

void DVRK_Arm::affix_tip_frame_rot(const tf::Quaternion &tf_quat){
    _rate_sleep();
    tip_ori_quat = tf_quat;
    affix_tip_frame(tip_pos, tip_ori_quat);
}

void DVRK_Arm::affix_tip_frame(const tf::Vector3 &pos, const tf::Quaternion &tf_quat){
    _rate_sleep();
    tip_trans.setOrigin(pos);
    tip_trans.setRotation(tf_quat);
}

void DVRK_Arm::affix_tip_frame(const tf::Vector3 &pos, const tf::Matrix3x3 &tf_mat){
    _rate_sleep();
    tip_pos = pos;
    tf_mat.getRotation(tip_ori_quat);
    affix_tip_frame(tip_pos, tip_ori_quat);
}

void DVRK_Arm::affix_tip_frame(const tf::Transform &trans){
    _rate_sleep();
    tip_trans = trans;
}

void DVRK_Arm::get_cur_position(double &x, double &y, double &z){
    _rate_sleep();
    x = ee_trans.getOrigin().getX();
    y = ee_trans.getOrigin().getY();
    z = ee_trans.getOrigin().getZ();
}

void DVRK_Arm::get_cur_position(tf::Vector3 &pos){
    _rate_sleep();
    pos = ee_trans.getOrigin();
}

void DVRK_Arm::get_cur_position(geometry_msgs::Point &pos){
    _rate_sleep();
    tf::pointTFToMsg(ee_trans.getOrigin(), pos);
}

void DVRK_Arm::get_cur_orientation(double &roll, double &pitch, double &yaw){
    _rate_sleep();
    tf::Matrix3x3(ee_trans.getRotation()).getRPY(roll, pitch, yaw);
}

void DVRK_Arm::get_cur_orientation(double &x, double &y, double &z, double &w){
    _rate_sleep();
    x = ee_trans.getRotation().getX();
    y = ee_trans.getRotation().getY();
    z = ee_trans.getRotation().getZ();
    w = ee_trans.getRotation().getW();
}

void DVRK_Arm::get_cur_orientation(tf::Quaternion &tf_quat){
    _rate_sleep();
    tf_quat = ee_trans.getRotation();
    tf_quat.normalize();
}

void DVRK_Arm::get_cur_orientation(geometry_msgs::Quaternion &gm_quat){
    _rate_sleep();
    tf::quaternionTFToMsg(ee_trans.getRotation(), gm_quat);
}

void DVRK_Arm::get_cur_orientation(tf::Matrix3x3 &mat){
    _rate_sleep();
    mat.setRotation(ee_trans.getRotation());
}

void DVRK_Arm::get_cur_pose(geometry_msgs::Pose &pose){
    _rate_sleep();
    pose.position.x = ee_trans.getOrigin().getX();
    pose.position.y = ee_trans.getOrigin().getY();
    pose.position.z = ee_trans.getOrigin().getZ();

    tf::quaternionTFToMsg(ee_trans.getRotation(), pose.orientation);
}

void DVRK_Arm::get_cur_transform(tf::Transform &trans){
    _rate_sleep();
    trans = ee_trans;
    trans.setRotation(trans.getRotation().normalized());
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
    ee_trans_cmd.setRotation(ee_ori_quat_cmd);
    move_arm_cartesian(ee_trans_cmd);
}

bool DVRK_Arm::set_orientation(const double &x, const double &y, const double &z, const double &w){
    ee_trans_cmd.setRotation(tf::Quaternion(x,y,z,w));
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

void DVRK_Arm::move_arm_cartesian(tf::Transform trans){
    trans = origin_trans.inverse() * trans * tip_trans.inverse();
    cmd_pose.pose.position.x = trans.getOrigin().getX();
    cmd_pose.pose.position.y = trans.getOrigin().getY();
    cmd_pose.pose.position.z = trans.getOrigin().getZ();
    tf::quaternionTFToMsg(trans.getRotation().normalized(), cmd_pose.pose.orientation);

    pose_pub.publish(cmd_pose.pose);
    ros::spinOnce();
    rate->sleep();
}

bool DVRK_Arm::set_force(const double &fx, const double &fy, const double &fz){
    ee_force_cmd.setX(fx);
    ee_force_cmd.setY(fy);
    ee_force_cmd.setZ(fz);

    set_arm_wrench(ee_force_cmd, ee_moment_cmd);
}

bool DVRK_Arm::set_moment(const double &nx, const double &ny, const double &nz){
    ee_moment_cmd.setX(nx);
    ee_moment_cmd.setY(ny);
    ee_moment_cmd.setZ(nz);

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
    _rate_sleep();
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
    _rate_sleep();
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
    _rate_sleep();
    if(_is_available()){
        if(strcmp(cur_state.data.c_str(), _m_jnt_pos_mode.c_str()) == 0){
            return true;
        }
    }
    else{
        return false;
    }
}

DVRK_Arm::~DVRK_Arm(){
}
