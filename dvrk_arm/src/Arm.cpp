#include "dvrk_arm/Arm.h"
DVRK_Arm::DVRK_Arm(const std::string &arm_name): DVRK_Bridge(arm_name), originFrame(new Frame), eeFrame(new Frame), afxdTipFrame(new Frame){
    init();
    assign_conversion_fcn(&DVRK_Arm::cisstPose_to_userTransform, this);

}

void DVRK_Arm::init(){
    handle_frames();
}

void DVRK_Arm::cisstPose_to_userTransform(const geometry_msgs::PoseStamped &pose){
    eeFrame->pos.setX(pose.pose.position.x);
    eeFrame->pos.setY(pose.pose.position.y);
    eeFrame->pos.setZ(pose.pose.position.z);
    tf::quaternionMsgToTF(pose.pose.orientation, eeFrame->rot_quat);

    eeFrame->trans.setOrigin(eeFrame->pos);
    eeFrame->trans.setRotation(eeFrame->rot_quat);
    eeFrame->trans = originFrame->trans * eeFrame->trans * afxdTipFrame->trans;

    eeFrame->pos = eeFrame->trans.getOrigin();
    eeFrame->rot_quat = eeFrame->trans.getRotation();
    eeFrame->rot_mat.setRotation(eeFrame->rot_quat);
}

void DVRK_Arm::set_origin_frame(const tf::Vector3 &pos, const tf::Matrix3x3 &tf_mat){
    _rate_sleep();
    tf_mat.getRotation(originFrame->rot_quat);
    set_origin_frame(pos, originFrame->rot_quat);
}

void DVRK_Arm::set_origin_frame(const tf::Vector3 &pos, const tf::Quaternion &tf_quat){
    _rate_sleep();
    originFrame->trans.setOrigin(pos);
    originFrame->trans.setRotation(tf_quat);
}

void DVRK_Arm::set_origin_frame(const tf::Transform &trans){
    _rate_sleep();
    originFrame->trans = trans;
}

void DVRK_Arm::set_origin_frame_pos(const double &x, const double &y, const double &z){
    _rate_sleep();
    originFrame->pos.setX(x);
    originFrame->pos.setY(y);
    originFrame->pos.setZ(z);

    set_origin_frame(originFrame->pos, originFrame->rot_quat);
}

void DVRK_Arm::set_origin_frame_pos(const geometry_msgs::Point &pos){
    _rate_sleep();
    originFrame->pos.setX(pos.x);
    originFrame->pos.setY(pos.y);
    originFrame->pos.setZ(pos.z);

    set_origin_frame(originFrame->pos, originFrame->rot_quat);
}

void DVRK_Arm::set_origin_frame_pos(const tf::Vector3 &pos){
    _rate_sleep();
    originFrame->pos = pos;

    set_origin_frame(originFrame->pos, originFrame->rot_quat);
}

void DVRK_Arm::set_origin_frame_rot(const double &roll, const double &pitch, const double &yaw){
    _rate_sleep();
    originFrame->rot_quat.setRPY(roll, pitch, yaw);

    set_origin_frame(originFrame->pos, originFrame->rot_quat);
}

void DVRK_Arm::set_origin_frame_rot(const tf::Quaternion &tf_quat){
    _rate_sleep();
    originFrame->rot_quat = tf_quat;

    set_origin_frame(originFrame->pos, originFrame->rot_quat);
}

void DVRK_Arm::set_origin_frame_rot(const geometry_msgs::Quaternion &gm_quat){
    _rate_sleep();
    tf::quaternionMsgToTF(gm_quat, originFrame->rot_quat);

    set_origin_frame(originFrame->pos, originFrame->rot_quat);
}

void DVRK_Arm::set_origin_frame_rot(const tf::Matrix3x3 &mat){
    _rate_sleep();
    mat.getRotation(originFrame->rot_quat);

    set_origin_frame(originFrame->pos, originFrame->rot_quat);
}

void DVRK_Arm::affix_tip_frame_pos(const double &x, const double &y, const double &z){
    _rate_sleep();
    afxdTipFrame->pos.setX(x);
    afxdTipFrame->pos.setY(y);
    afxdTipFrame->pos.setZ(z);
    affix_tip_frame(afxdTipFrame->pos, afxdTipFrame->rot_quat);
}

void DVRK_Arm::affix_tip_frame_pos(const geometry_msgs::Point &pos){
    _rate_sleep();
    afxdTipFrame->pos.setX(pos.x);
    afxdTipFrame->pos.setY(pos.y);
    afxdTipFrame->pos.setZ(pos.z);
    affix_tip_frame(afxdTipFrame->pos, afxdTipFrame->rot_quat);
}

void DVRK_Arm::affix_tip_frame_pos(const tf::Vector3 &pos){
    _rate_sleep();
    afxdTipFrame->pos = pos;
    affix_tip_frame(afxdTipFrame->pos, afxdTipFrame->rot_quat);
}

void DVRK_Arm::affix_tip_frame_rot(const double &roll, const double &pitch, const double &yaw){
    _rate_sleep();
    afxdTipFrame->rot_quat.setRPY(roll, pitch, yaw);
    affix_tip_frame(afxdTipFrame->pos, afxdTipFrame->rot_quat);
}

void DVRK_Arm::affix_tip_frame_rot(const double &quat_x, const double &quat_y, const double &quat_z, const double &quat_w){
    _rate_sleep();
    afxdTipFrame->rot_quat.setX(quat_x);
    afxdTipFrame->rot_quat.setY(quat_y);
    afxdTipFrame->rot_quat.setZ(quat_z);
    afxdTipFrame->rot_quat.setW(quat_w);
    affix_tip_frame(afxdTipFrame->pos, afxdTipFrame->rot_quat);
}

void DVRK_Arm::affix_tip_frame_rot(const geometry_msgs::Quaternion &gm_quat){
    _rate_sleep();
    tf::quaternionMsgToTF(gm_quat, afxdTipFrame->rot_quat);
    affix_tip_frame(afxdTipFrame->pos, afxdTipFrame->rot_quat);
}

void DVRK_Arm::affix_tip_frame_rot(const tf::Quaternion &tf_quat){
    _rate_sleep();
    afxdTipFrame->rot_quat = tf_quat;
    affix_tip_frame(afxdTipFrame->pos, afxdTipFrame->rot_quat);
}

void DVRK_Arm::affix_tip_frame(const tf::Vector3 &pos, const tf::Quaternion &tf_quat){
    _rate_sleep();
    afxdTipFrame->trans.setOrigin(pos);
    afxdTipFrame->trans.setRotation(tf_quat);
}

void DVRK_Arm::affix_tip_frame(const tf::Vector3 &pos, const tf::Matrix3x3 &tf_mat){
    _rate_sleep();
    afxdTipFrame->pos = pos;
    tf_mat.getRotation(afxdTipFrame->rot_quat);
    affix_tip_frame(afxdTipFrame->pos, afxdTipFrame->rot_quat);
}

void DVRK_Arm::affix_tip_frame(const tf::Transform &trans){
    _rate_sleep();
    afxdTipFrame->trans = trans;
}

void DVRK_Arm::get_cur_position(double &x, double &y, double &z){
    _rate_sleep();
    x = eeFrame->trans.getOrigin().getX();
    y = eeFrame->trans.getOrigin().getY();
    z = eeFrame->trans.getOrigin().getZ();
}

void DVRK_Arm::get_cur_position(tf::Vector3 &pos){
    _rate_sleep();
    pos = eeFrame->trans.getOrigin();
}

void DVRK_Arm::get_cur_position(geometry_msgs::Point &pos){
    _rate_sleep();
    tf::pointTFToMsg(eeFrame->trans.getOrigin(), pos);
}

void DVRK_Arm::get_cur_orientation(double &roll, double &pitch, double &yaw){
    _rate_sleep();
    tf::Matrix3x3(eeFrame->trans.getRotation()).getRPY(roll, pitch, yaw);
}

void DVRK_Arm::get_cur_orientation(double &x, double &y, double &z, double &w){
    _rate_sleep();
    x = eeFrame->trans.getRotation().getX();
    y = eeFrame->trans.getRotation().getY();
    z = eeFrame->trans.getRotation().getZ();
    w = eeFrame->trans.getRotation().getW();
}

void DVRK_Arm::get_cur_orientation(tf::Quaternion &tf_quat){
    _rate_sleep();
    tf_quat = eeFrame->trans.getRotation();
    tf_quat.normalize();
}

void DVRK_Arm::get_cur_orientation(geometry_msgs::Quaternion &gm_quat){
    _rate_sleep();
    tf::quaternionTFToMsg(eeFrame->trans.getRotation(), gm_quat);
}

void DVRK_Arm::get_cur_orientation(tf::Matrix3x3 &mat){
    _rate_sleep();
    mat.setRotation(eeFrame->trans.getRotation());
}

void DVRK_Arm::get_cur_pose(geometry_msgs::Pose &pose){
    _rate_sleep();
    pose.position.x = eeFrame->trans.getOrigin().getX();
    pose.position.y = eeFrame->trans.getOrigin().getY();
    pose.position.z = eeFrame->trans.getOrigin().getZ();

    tf::quaternionTFToMsg(eeFrame->trans.getRotation(), pose.orientation);
}

void DVRK_Arm::get_cur_transform(tf::Transform &trans){
    _rate_sleep();
    trans = eeFrame->trans;
    trans.setRotation(trans.getRotation().normalized());
}

bool DVRK_Arm::set_position(const double &x, const double &y, const double &z){
    eeCmd.trans.setOrigin(tf::Vector3(x,y,z));
    move_arm_cartesian(eeCmd.trans);
}

bool DVRK_Arm::set_position(const geometry_msgs::Point &pos){
    eeCmd.trans.setOrigin(tf::Vector3(pos.x, pos.y, pos.z));
    move_arm_cartesian(eeCmd.trans);
}

bool DVRK_Arm::set_position(const tf::Vector3 &pos){
    eeCmd.trans.setOrigin(pos);
    move_arm_cartesian(eeCmd.trans);
}

bool DVRK_Arm::set_orientation(const double &roll, const double &pitch, const double &yaw){
    eeCmd.rot_quat.setRPY(roll, pitch, yaw);
    eeCmd.trans.setRotation(eeCmd.rot_quat);
    move_arm_cartesian(eeCmd.trans);
}

bool DVRK_Arm::set_orientation(const double &x, const double &y, const double &z, const double &w){
    eeCmd.trans.setRotation(tf::Quaternion(x,y,z,w));
    move_arm_cartesian(eeCmd.trans);
}

bool DVRK_Arm::set_orientation(const tf::Quaternion &tf_quat){
    eeCmd.trans.setRotation(tf_quat);
    move_arm_cartesian(eeCmd.trans);
}

bool DVRK_Arm::set_orientation(const geometry_msgs::Quaternion &gm_quat){
    eeCmd.trans.setRotation(tf::Quaternion(gm_quat.x, gm_quat.y, gm_quat.z, gm_quat.w));
    move_arm_cartesian(eeCmd.trans);
}

bool DVRK_Arm::set_orientation(const tf::Matrix3x3 &mat){
    mat.getRotation(eeCmd.rot_quat);
    eeCmd.trans.setRotation(eeCmd.rot_quat);
    move_arm_cartesian(eeCmd.trans);
}

bool DVRK_Arm::set_pose(geometry_msgs::PoseStamped &pose){
    eeCmd.trans.setOrigin(tf::Vector3(pose.pose.position.x,
                           pose.pose.position.y,
                           pose.pose.position.z));

    eeCmd.trans.setRotation(tf::Quaternion(pose.pose.orientation.x,
                             pose.pose.orientation.y,
                             pose.pose.orientation.z,
                             pose.pose.orientation.w));
    move_arm_cartesian(eeCmd.trans);
}

bool DVRK_Arm::set_transform(tf::Transform &trans){
    eeCmd.trans = trans;
    move_arm_cartesian(eeCmd.trans);
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
    trans = originFrame->trans.inverse() * trans * afxdTipFrame->trans.inverse();
    cmd_pose.pose.position.x = trans.getOrigin().getX();
    cmd_pose.pose.position.y = trans.getOrigin().getY();
    cmd_pose.pose.position.z = trans.getOrigin().getZ();
    tf::quaternionTFToMsg(trans.getRotation().normalized(), cmd_pose.pose.orientation);

    pose_pub.publish(cmd_pose.pose);
    ros::spinOnce();
    rate->sleep();
}

bool DVRK_Arm::set_force(const double &fx, const double &fy, const double &fz){
    eeCmd.force.setX(fx);
    eeCmd.force.setY(fy);
    eeCmd.force.setZ(fz);

    set_arm_wrench(eeCmd.force, eeCmd.moment);
}

bool DVRK_Arm::set_moment(const double &nx, const double &ny, const double &nz){
    eeCmd.moment.setX(nx);
    eeCmd.moment.setY(ny);
    eeCmd.moment.setZ(nz);

    set_arm_wrench(eeCmd.force, eeCmd.moment);
}

bool DVRK_Arm::set_wrench(const double &fx,const double &fy,const double &fz,const double &nx,const double &ny,const double &nz){

    eeCmd.force.setX(fx);
    eeCmd.force.setY(fy);
    eeCmd.force.setZ(fz);

    eeCmd.moment.setX(nx);
    eeCmd.moment.setY(ny);
    eeCmd.moment.setZ(nz);

    set_arm_wrench(eeCmd.force, eeCmd.moment);
}

void DVRK_Arm::set_arm_wrench(tf::Vector3 &force, tf::Vector3 &moment){
    if(_clutch_pressed == true){
        force.setZero();
        moment.setZero();
    }
    originFrame->rot_mat.setRotation(originFrame->trans.getRotation());
    tf::vector3TFToMsg(originFrame->rot_mat.inverse() * force, cmd_wrench.force);
    tf::vector3TFToMsg(originFrame->rot_mat.inverse() * moment, cmd_wrench.torque);
    //ROS_INFO("Ori F %f %f %f", force.x(),force.y(),force.z());
    //ROS_INFO("Ori M %f %f %f", moment.x(),moment.y(),moment.z());
    //ROS_INFO("Cmd F %f %f %f", cmd_wrench.force.x,cmd_wrench.force.y,cmd_wrench.force.z);
    //ROS_INFO("Cmd M %f %f %f", cmd_wrench.torque.x,cmd_wrench.torque.y,cmd_wrench.torque.z);
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

void DVRK_Arm::handle_frames(){
}

DVRK_Arm::~DVRK_Arm(){
}
