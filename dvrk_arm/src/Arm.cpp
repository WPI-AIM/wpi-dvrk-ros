#include "dvrk_arm/Arm.h"
DVRK_Arm::DVRK_Arm(const std::string &arm_name): DVRK_Bridge(arm_name), originFramePtr(new Frame), eeFramePtr(new Frame), afxdTipFramePtr(new Frame){
    init();
    assign_conversion_fcn(&DVRK_Arm::cisstPose_to_userTransform, this);
    frameptrVec.push_back(originFramePtr);
    frameptrVec.push_back(eeFramePtr);
    frameptrVec.push_back(afxdTipFramePtr);

}

void DVRK_Arm::init(){
    handle_frames();
}

void DVRK_Arm::cisstPose_to_userTransform(const geometry_msgs::PoseStamped &pose){
    eeFramePtr->pos.setX(pose.pose.position.x);
    eeFramePtr->pos.setY(pose.pose.position.y);
    eeFramePtr->pos.setZ(pose.pose.position.z);
    tf::quaternionMsgToTF(pose.pose.orientation, eeFramePtr->rot_quat);

    eeFramePtr->trans.setOrigin(eeFramePtr->pos);
    eeFramePtr->trans.setRotation(eeFramePtr->rot_quat);
    eeFramePtr->trans = originFramePtr->trans * eeFramePtr->trans * afxdTipFramePtr->trans;

    eeFramePtr->pos = eeFramePtr->trans.getOrigin();
    eeFramePtr->rot_quat = eeFramePtr->trans.getRotation();
    eeFramePtr->rot_mat.setRotation(eeFramePtr->rot_quat);
}

void DVRK_Arm::set_origin_frame(const tf::Vector3 &pos, const tf::Matrix3x3 &tf_mat){
    _rate_sleep();
    tf_mat.getRotation(originFramePtr->rot_quat);
    set_origin_frame(pos, originFramePtr->rot_quat);
}

void DVRK_Arm::set_origin_frame(const tf::Vector3 &pos, const tf::Quaternion &tf_quat){
    _rate_sleep();
    originFramePtr->trans.setOrigin(pos);
    originFramePtr->trans.setRotation(tf_quat);
}

void DVRK_Arm::set_origin_frame(const tf::Transform &trans){
    _rate_sleep();
    originFramePtr->trans = trans;
}

void DVRK_Arm::set_origin_frame_pos(const double &x, const double &y, const double &z){
    _rate_sleep();
    originFramePtr->pos.setX(x);
    originFramePtr->pos.setY(y);
    originFramePtr->pos.setZ(z);

    set_origin_frame(originFramePtr->pos, originFramePtr->rot_quat);
}

void DVRK_Arm::set_origin_frame_pos(const geometry_msgs::Point &pos){
    _rate_sleep();
    originFramePtr->pos.setX(pos.x);
    originFramePtr->pos.setY(pos.y);
    originFramePtr->pos.setZ(pos.z);

    set_origin_frame(originFramePtr->pos, originFramePtr->rot_quat);
}

void DVRK_Arm::set_origin_frame_pos(const tf::Vector3 &pos){
    _rate_sleep();
    originFramePtr->pos = pos;

    set_origin_frame(originFramePtr->pos, originFramePtr->rot_quat);
}

void DVRK_Arm::set_origin_frame_rot(const double &roll, const double &pitch, const double &yaw){
    _rate_sleep();
    originFramePtr->rot_quat.setRPY(roll, pitch, yaw);

    set_origin_frame(originFramePtr->pos, originFramePtr->rot_quat);
}

void DVRK_Arm::set_origin_frame_rot(const tf::Quaternion &tf_quat){
    _rate_sleep();
    originFramePtr->rot_quat = tf_quat;

    set_origin_frame(originFramePtr->pos, originFramePtr->rot_quat);
}

void DVRK_Arm::set_origin_frame_rot(const geometry_msgs::Quaternion &gm_quat){
    _rate_sleep();
    tf::quaternionMsgToTF(gm_quat, originFramePtr->rot_quat);

    set_origin_frame(originFramePtr->pos, originFramePtr->rot_quat);
}

void DVRK_Arm::set_origin_frame_rot(const tf::Matrix3x3 &mat){
    _rate_sleep();
    mat.getRotation(originFramePtr->rot_quat);

    set_origin_frame(originFramePtr->pos, originFramePtr->rot_quat);
}

void DVRK_Arm::affix_tip_frame_pos(const double &x, const double &y, const double &z){
    _rate_sleep();
    afxdTipFramePtr->pos.setX(x);
    afxdTipFramePtr->pos.setY(y);
    afxdTipFramePtr->pos.setZ(z);
    affix_tip_frame(afxdTipFramePtr->pos, afxdTipFramePtr->rot_quat);
}

void DVRK_Arm::affix_tip_frame_pos(const geometry_msgs::Point &pos){
    _rate_sleep();
    afxdTipFramePtr->pos.setX(pos.x);
    afxdTipFramePtr->pos.setY(pos.y);
    afxdTipFramePtr->pos.setZ(pos.z);
    affix_tip_frame(afxdTipFramePtr->pos, afxdTipFramePtr->rot_quat);
}

void DVRK_Arm::affix_tip_frame_pos(const tf::Vector3 &pos){
    _rate_sleep();
    afxdTipFramePtr->pos = pos;
    affix_tip_frame(afxdTipFramePtr->pos, afxdTipFramePtr->rot_quat);
}

void DVRK_Arm::affix_tip_frame_rot(const double &roll, const double &pitch, const double &yaw){
    _rate_sleep();
    afxdTipFramePtr->rot_quat.setRPY(roll, pitch, yaw);
    affix_tip_frame(afxdTipFramePtr->pos, afxdTipFramePtr->rot_quat);
}

void DVRK_Arm::affix_tip_frame_rot(const double &quat_x, const double &quat_y, const double &quat_z, const double &quat_w){
    _rate_sleep();
    afxdTipFramePtr->rot_quat.setX(quat_x);
    afxdTipFramePtr->rot_quat.setY(quat_y);
    afxdTipFramePtr->rot_quat.setZ(quat_z);
    afxdTipFramePtr->rot_quat.setW(quat_w);
    affix_tip_frame(afxdTipFramePtr->pos, afxdTipFramePtr->rot_quat);
}

void DVRK_Arm::affix_tip_frame_rot(const geometry_msgs::Quaternion &gm_quat){
    _rate_sleep();
    tf::quaternionMsgToTF(gm_quat, afxdTipFramePtr->rot_quat);
    affix_tip_frame(afxdTipFramePtr->pos, afxdTipFramePtr->rot_quat);
}

void DVRK_Arm::affix_tip_frame_rot(const tf::Quaternion &tf_quat){
    _rate_sleep();
    afxdTipFramePtr->rot_quat = tf_quat;
    affix_tip_frame(afxdTipFramePtr->pos, afxdTipFramePtr->rot_quat);
}

void DVRK_Arm::affix_tip_frame(const tf::Vector3 &pos, const tf::Quaternion &tf_quat){
    _rate_sleep();
    afxdTipFramePtr->trans.setOrigin(pos);
    afxdTipFramePtr->trans.setRotation(tf_quat);
}

void DVRK_Arm::affix_tip_frame(const tf::Vector3 &pos, const tf::Matrix3x3 &tf_mat){
    _rate_sleep();
    afxdTipFramePtr->pos = pos;
    tf_mat.getRotation(afxdTipFramePtr->rot_quat);
    affix_tip_frame(afxdTipFramePtr->pos, afxdTipFramePtr->rot_quat);
}

void DVRK_Arm::affix_tip_frame(const tf::Transform &trans){
    _rate_sleep();
    afxdTipFramePtr->trans = trans;
}

void DVRK_Arm::get_cur_position(double &x, double &y, double &z){
    _rate_sleep();
    x = eeFramePtr->trans.getOrigin().getX();
    y = eeFramePtr->trans.getOrigin().getY();
    z = eeFramePtr->trans.getOrigin().getZ();
}

void DVRK_Arm::get_cur_position(tf::Vector3 &pos){
    _rate_sleep();
    pos = eeFramePtr->trans.getOrigin();
}

void DVRK_Arm::get_cur_position(geometry_msgs::Point &pos){
    _rate_sleep();
    tf::pointTFToMsg(eeFramePtr->trans.getOrigin(), pos);
}

void DVRK_Arm::get_cur_orientation(double &roll, double &pitch, double &yaw){
    _rate_sleep();
    tf::Matrix3x3(eeFramePtr->trans.getRotation()).getRPY(roll, pitch, yaw);
}

void DVRK_Arm::get_cur_orientation(double &x, double &y, double &z, double &w){
    _rate_sleep();
    x = eeFramePtr->trans.getRotation().getX();
    y = eeFramePtr->trans.getRotation().getY();
    z = eeFramePtr->trans.getRotation().getZ();
    w = eeFramePtr->trans.getRotation().getW();
}

void DVRK_Arm::get_cur_orientation(tf::Quaternion &tf_quat){
    _rate_sleep();
    tf_quat = eeFramePtr->trans.getRotation();
    tf_quat.normalize();
}

void DVRK_Arm::get_cur_orientation(geometry_msgs::Quaternion &gm_quat){
    _rate_sleep();
    tf::quaternionTFToMsg(eeFramePtr->trans.getRotation(), gm_quat);
}

void DVRK_Arm::get_cur_orientation(tf::Matrix3x3 &mat){
    _rate_sleep();
    mat.setRotation(eeFramePtr->trans.getRotation());
}

void DVRK_Arm::get_cur_pose(geometry_msgs::Pose &pose){
    _rate_sleep();
    pose.position.x = eeFramePtr->trans.getOrigin().getX();
    pose.position.y = eeFramePtr->trans.getOrigin().getY();
    pose.position.z = eeFramePtr->trans.getOrigin().getZ();

    tf::quaternionTFToMsg(eeFramePtr->trans.getRotation(), pose.orientation);
}

void DVRK_Arm::get_cur_transform(tf::Transform &trans){
    _rate_sleep();
    trans = eeFramePtr->trans;
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
    trans = originFramePtr->trans.inverse() * trans * afxdTipFramePtr->trans.inverse();
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
    originFramePtr->rot_mat.setRotation(originFramePtr->trans.getRotation());
    tf::vector3TFToMsg(originFramePtr->rot_mat.inverse() * force, cmd_wrench.force);
    tf::vector3TFToMsg(originFramePtr->rot_mat.inverse() * moment, cmd_wrench.torque);
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
    for(frameIter = frameptrVec.begin(); frameIter !=frameptrVec.end(); frameIter++){
        double x,y,z;
        x = (*frameIter)->trans.getOrigin().getX();
        y = (*frameIter)->trans.getOrigin().getY();
        z = (*frameIter)->trans.getOrigin().getZ();

        double qx, qy, qz, qw;
        qx = (*frameIter)->trans.getRotation().getX();
        qy = (*frameIter)->trans.getRotation().getY();
        qz = (*frameIter)->trans.getRotation().getZ();
        qw = (*frameIter)->trans.getRotation().getW();
        if (isnan(x) || isnan(y) ||  isnan(z)){
            (*frameIter)->trans.setOrigin(tf::Vector3(0,0,0));
            ROS_ERROR("Origin of frame is NAN, setting origin to (0,0,0)");
        }
        if (isnan(qx) || isnan(qy) ||  isnan(qz) || isnan(qw)){
            (*frameIter)->trans.setRotation(tf::Quaternion().getIdentity());
            ROS_ERROR("Rotation of frame is NAN, setting rotation to (0,0,0)");
        }
        //Normalized the rotation quaternion;
        (*frameIter)->trans.getRotation() = (*frameIter)->trans.getRotation().normalized();
        (*frameIter)->pos = (*frameIter)->trans.getOrigin();
        (*frameIter)->rot_quat = (*frameIter)->trans.getRotation();
        (*frameIter)->rot_mat.setRotation((*frameIter)->rot_quat);
    }
}

DVRK_Arm::~DVRK_Arm(){
}
