#include "dvrk_arm/Arm.h"
DVRK_Arm::DVRK_Arm(const std::string &arm_name): DVRK_Bridge(arm_name){
    originFramePtr.reset(new Frame);
    eeFramePtr.reset(new Frame);
    afxdTipFramePtr.reset(new Frame);
    freeFramePtr.reset(new Frame);


    frameptrVec.push_back(originFramePtr);
    frameptrVec.push_back(eeFramePtr);
    frameptrVec.push_back(afxdTipFramePtr);

    init();
    poseConversion.assign_conversion_fcn(&DVRK_Arm::cisstPose_to_userTransform, this);
    jointConversion.assign_conversion_fcn(&DVRK_Arm::cisstJoint_to_userJoint, this);
    wrenchConversion.assign_conversion_fcn(&DVRK_Arm::cisstWrench_to_userWrench, this);

}

void DVRK_Arm::init(){
}

void DVRK_Arm::cisstPose_to_userTransform(const geometry_msgs::PoseStamped &pose){
    freeFramePtr->pos.setX(pose.pose.position.x);
    freeFramePtr->pos.setY(pose.pose.position.y);
    freeFramePtr->pos.setZ(pose.pose.position.z);
    tf::quaternionMsgToTF(pose.pose.orientation, freeFramePtr->rot_quat);

    freeFramePtr->trans.setOrigin(freeFramePtr->pos);
    freeFramePtr->trans.setRotation(freeFramePtr->rot_quat);
    freeFramePtr->trans = originFramePtr->trans * freeFramePtr->trans * afxdTipFramePtr->trans;
    eeFramePtr->trans = freeFramePtr->trans;

    handle_frames();
}

void DVRK_Arm::cisstJoint_to_userJoint(const sensor_msgs::JointState &jnt){

}

void DVRK_Arm::cisstWrench_to_userWrench(const geometry_msgs::WrenchStamped &wrench){

}

void DVRK_Arm::set_origin_frame(const tf::Vector3 &pos, const tf::Matrix3x3 &tf_mat){
    tf_mat.getRotation(originFramePtr->rot_quat);
    set_origin_frame(pos, originFramePtr->rot_quat);
}

void DVRK_Arm::set_origin_frame(const tf::Vector3 &pos, const tf::Quaternion &tf_quat){
    originFramePtr->trans.setOrigin(pos);
    originFramePtr->trans.setRotation(tf_quat);
}

void DVRK_Arm::set_origin_frame(const tf::Transform &trans){
    originFramePtr->trans = trans;
}

void DVRK_Arm::set_origin_frame_pos(const double &x, const double &y, const double &z){
    originFramePtr->pos.setX(x);
    originFramePtr->pos.setY(y);
    originFramePtr->pos.setZ(z);

    set_origin_frame(originFramePtr->pos, originFramePtr->rot_quat);
}

void DVRK_Arm::set_origin_frame_pos(const geometry_msgs::Point &pos){
    originFramePtr->pos.setX(pos.x);
    originFramePtr->pos.setY(pos.y);
    originFramePtr->pos.setZ(pos.z);

    set_origin_frame(originFramePtr->pos, originFramePtr->rot_quat);
}

void DVRK_Arm::set_origin_frame_pos(const tf::Vector3 &pos){
    originFramePtr->pos = pos;

    set_origin_frame(originFramePtr->pos, originFramePtr->rot_quat);
}

void DVRK_Arm::set_origin_frame_rot(const double &roll, const double &pitch, const double &yaw){
    originFramePtr->rot_quat.setRPY(roll, pitch, yaw);

    set_origin_frame(originFramePtr->pos, originFramePtr->rot_quat);
}

void DVRK_Arm::set_origin_frame_rot(const tf::Quaternion &tf_quat){
    originFramePtr->rot_quat = tf_quat;

    set_origin_frame(originFramePtr->pos, originFramePtr->rot_quat);
}

void DVRK_Arm::set_origin_frame_rot(const geometry_msgs::Quaternion &gm_quat){
    tf::quaternionMsgToTF(gm_quat, originFramePtr->rot_quat);

    set_origin_frame(originFramePtr->pos, originFramePtr->rot_quat);
}

void DVRK_Arm::set_origin_frame_rot(const tf::Matrix3x3 &mat){
    mat.getRotation(originFramePtr->rot_quat);

    set_origin_frame(originFramePtr->pos, originFramePtr->rot_quat);
}

void DVRK_Arm::affix_tip_frame_pos(const double &x, const double &y, const double &z){
    afxdTipFramePtr->pos.setX(x);
    afxdTipFramePtr->pos.setY(y);
    afxdTipFramePtr->pos.setZ(z);
    affix_tip_frame(afxdTipFramePtr->pos, afxdTipFramePtr->rot_quat);
}

void DVRK_Arm::affix_tip_frame_pos(const geometry_msgs::Point &pos){
    afxdTipFramePtr->pos.setX(pos.x);
    afxdTipFramePtr->pos.setY(pos.y);
    afxdTipFramePtr->pos.setZ(pos.z);
    affix_tip_frame(afxdTipFramePtr->pos, afxdTipFramePtr->rot_quat);
}

void DVRK_Arm::affix_tip_frame_pos(const tf::Vector3 &pos){
    afxdTipFramePtr->pos = pos;
    affix_tip_frame(afxdTipFramePtr->pos, afxdTipFramePtr->rot_quat);
}

void DVRK_Arm::affix_tip_frame_rot(const double &roll, const double &pitch, const double &yaw){
    afxdTipFramePtr->rot_quat.setRPY(roll, pitch, yaw);
    affix_tip_frame(afxdTipFramePtr->pos, afxdTipFramePtr->rot_quat);
}

void DVRK_Arm::affix_tip_frame_rot(const double &quat_x, const double &quat_y, const double &quat_z, const double &quat_w){
    afxdTipFramePtr->rot_quat.setX(quat_x);
    afxdTipFramePtr->rot_quat.setY(quat_y);
    afxdTipFramePtr->rot_quat.setZ(quat_z);
    afxdTipFramePtr->rot_quat.setW(quat_w);
    affix_tip_frame(afxdTipFramePtr->pos, afxdTipFramePtr->rot_quat);
}

void DVRK_Arm::affix_tip_frame_rot(const geometry_msgs::Quaternion &gm_quat){
    tf::quaternionMsgToTF(gm_quat, afxdTipFramePtr->rot_quat);
    affix_tip_frame(afxdTipFramePtr->pos, afxdTipFramePtr->rot_quat);
}

void DVRK_Arm::affix_tip_frame_rot(const tf::Quaternion &tf_quat){
    afxdTipFramePtr->rot_quat = tf_quat;
    affix_tip_frame(afxdTipFramePtr->pos, afxdTipFramePtr->rot_quat);
}

void DVRK_Arm::affix_tip_frame(const tf::Vector3 &pos, const tf::Quaternion &tf_quat){
    afxdTipFramePtr->trans.setOrigin(pos);
    afxdTipFramePtr->trans.setRotation(tf_quat);
}

void DVRK_Arm::affix_tip_frame(const tf::Vector3 &pos, const tf::Matrix3x3 &tf_mat){
    afxdTipFramePtr->pos = pos;
    tf_mat.getRotation(afxdTipFramePtr->rot_quat);
    affix_tip_frame(afxdTipFramePtr->pos, afxdTipFramePtr->rot_quat);
}

void DVRK_Arm::affix_tip_frame(const tf::Transform &trans){
    afxdTipFramePtr->trans = trans;
}

void DVRK_Arm::get_cur_position(double &x, double &y, double &z){
    x = eeFramePtr->trans.getOrigin().getX();
    y = eeFramePtr->trans.getOrigin().getY();
    z = eeFramePtr->trans.getOrigin().getZ();
}

void DVRK_Arm::get_cur_position(tf::Vector3 &pos){
    pos = eeFramePtr->trans.getOrigin();
}

void DVRK_Arm::get_cur_position(geometry_msgs::Point &pos){
    tf::pointTFToMsg(eeFramePtr->trans.getOrigin(), pos);
}

void DVRK_Arm::get_cur_orientation(double &roll, double &pitch, double &yaw){
    tf::Matrix3x3(eeFramePtr->trans.getRotation()).getRPY(roll, pitch, yaw);
}

void DVRK_Arm::get_cur_orientation(double &x, double &y, double &z, double &w){
    x = eeFramePtr->trans.getRotation().getX();
    y = eeFramePtr->trans.getRotation().getY();
    z = eeFramePtr->trans.getRotation().getZ();
    w = eeFramePtr->trans.getRotation().getW();
}

void DVRK_Arm::get_cur_orientation(tf::Quaternion &tf_quat){
    tf_quat = eeFramePtr->trans.getRotation();
    tf_quat.normalize();
}

void DVRK_Arm::get_cur_orientation(geometry_msgs::Quaternion &gm_quat){
    tf::quaternionTFToMsg(eeFramePtr->trans.getRotation(), gm_quat);
}

void DVRK_Arm::get_cur_orientation(tf::Matrix3x3 &mat){
    mat.setRotation(eeFramePtr->trans.getRotation());
}

void DVRK_Arm::get_cur_pose(geometry_msgs::Pose &pose){
    pose.position.x = eeFramePtr->trans.getOrigin().getX();
    pose.position.y = eeFramePtr->trans.getOrigin().getY();
    pose.position.z = eeFramePtr->trans.getOrigin().getZ();

    tf::quaternionTFToMsg(eeFramePtr->trans.getRotation(), pose.orientation);
}

void DVRK_Arm::get_cur_transform(tf::Transform &trans){
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

void DVRK_Arm::set_mode(const std::string &state, bool lock_wrench_ori){
    set_cur_mode(state, lock_wrench_ori);
}

void DVRK_Arm::move_arm_cartesian(tf::Transform trans){
    geometry_msgs::PoseStamped cmd_pose;
    trans = originFramePtr->trans.inverse() * trans * afxdTipFramePtr->trans.inverse();
    cmd_pose.pose.position.x = trans.getOrigin().getX();
    cmd_pose.pose.position.y = trans.getOrigin().getY();
    cmd_pose.pose.position.z = trans.getOrigin().getZ();
    tf::quaternionTFToMsg(trans.getRotation().normalized(), cmd_pose.pose.orientation);

    set_cur_pose(cmd_pose);
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
    geometry_msgs::Wrench cmd_wrench;
    tf::vector3TFToMsg(originFramePtr->rot_mat.inverse() * force, cmd_wrench.force);
    tf::vector3TFToMsg(originFramePtr->rot_mat.inverse() * moment, cmd_wrench.torque);
    //ROS_INFO("Ori F %f %f %f", force.x(),force.y(),force.z());
    //ROS_INFO("Ori M %f %f %f", moment.x(),moment.y(),moment.z());
    //ROS_INFO("Cmd F %f %f %f", cmd_wrench.force.x,cmd_wrench.force.y,cmd_wrench.force.z);
    //ROS_INFO("Cmd M %f %f %f", cmd_wrench.torque.x,cmd_wrench.torque.y,cmd_wrench.torque.z);
    set_cur_wrench(cmd_wrench);
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
        //Normalize the rotation quaternion;
        (*frameIter)->trans.getRotation() = (*frameIter)->trans.getRotation().normalized();
        //Setting the pos, quat and rot_mat members of the struct so all of them have the same data as trans
        (*frameIter)->pos = (*frameIter)->trans.getOrigin();
        (*frameIter)->rot_quat = (*frameIter)->trans.getRotation();
        (*frameIter)->rot_mat.setRotation((*frameIter)->rot_quat);
    }
}

DVRK_Arm::~DVRK_Arm(){
}
