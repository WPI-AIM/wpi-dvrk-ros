#ifndef CDVRK_ArmH
#define CDVRK_ArmH

#include "Bridge.h"
#include "tf/tf.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "boost/shared_ptr.hpp"
#include "dvrk_arm/Frame.h"

struct Command: public Frame{
public:
    Command(){
        force.setZero();
        moment.setZero();
    }
    ~Command(){}
    tf::Vector3 force;
    tf::Vector3 moment;
};


class DVRK_Arm: public DVRK_Bridge{
public:
    DVRK_Arm(const std::string &arm_name);
    ~DVRK_Arm();

    void set_origin_frame_pos(const double &x, const double &y, const double &);
    void set_origin_frame_pos(const geometry_msgs::Point &pos);
    void set_origin_frame_pos(const tf::Vector3 &pos);

    void set_origin_frame_rot(const double &roll, const double &pitch, const double &yaw);
    void set_origin_frame_rot(const tf::Quaternion &tf_quat);
    void set_origin_frame_rot(const geometry_msgs::Quaternion &gm_quat);
    void set_origin_frame_rot(const tf::Matrix3x3 &mat);

    void set_origin_frame(const tf::Vector3 &pos, const tf::Quaternion &tf_quat);
    void set_origin_frame(const tf::Vector3 &pos, const tf::Matrix3x3 &tf_mat);
    void set_origin_frame(const tf::Transform &trans);

    void affix_tip_frame_pos(const double &x, const double &y, const double &z);
    void affix_tip_frame_pos(const tf::Vector3 &pos);
    void affix_tip_frame_pos(const geometry_msgs::Point &pos);

    void affix_tip_frame_rot(const tf::Quaternion &tf_quat);
    void affix_tip_frame_rot(const geometry_msgs::Quaternion &gm_quat);
    void affix_tip_frame_rot(const double &quat_x, const double &quat_y, const double &quat_z, const double &quat_w);
    void affix_tip_frame_rot(const double &roll, const double &pitch, const double &yaw);

    void affix_tip_frame(const tf::Vector3 &pos, const tf::Quaternion &tf_quat);
    void affix_tip_frame(const tf::Vector3 &pos, const tf::Matrix3x3 &tf_mat);
    void affix_tip_frame(const tf::Transform &trans);

    bool set_force(const double &fx,const double &fy,const double &fz);
    bool set_moment(const double &nx,const double &ny,const double &nz);
    bool set_wrench(const double &fx,const double &fy,const double &fz,const double &nx,const double &ny,const double &nz);

    bool set_position(const double &x, const double &y, const double &z);
    bool set_position(const geometry_msgs::Point &pos);
    bool set_position(const tf::Vector3 &pos);

    bool set_orientation(const double &roll, const double &pitch, const double &yaw);
    bool set_orientation(const double &x, const double &y, const double &z, const double &w);
    bool set_orientation(const tf::Quaternion &tf_quat);
    bool set_orientation(const geometry_msgs::Quaternion &gm_quat);
    bool set_orientation(const tf::Matrix3x3 &mat);

    bool set_pose(geometry_msgs::PoseStamped &pose);
    bool set_transform(tf::Transform &trans);

    void get_cur_position(double &x, double &y, double &z);
    void get_cur_position(tf::Vector3 &pos);
    void get_cur_position(geometry_msgs::Point &pos);

    void get_cur_orientation(double &roll, double &pitch, double &yaw);
    void get_cur_orientation(double &x, double &y, double &z, double &w);
    void get_cur_orientation(tf::Quaternion &tf_quat);
    void get_cur_orientation(geometry_msgs::Quaternion &gm_quat);
    void get_cur_orientation(tf::Matrix3x3 &mat);

    void get_cur_pose(geometry_msgs::Pose &pose);
    void get_cur_transform(tf::Transform &trans);

    void set_mode(const std::string &state, bool lock_wrench_ori = true);

private:

    void init();
    void handle_frames();
    void cisstPose_to_userTransform(const geometry_msgs::PoseStamped &pose);
    void userPose_to_cisstPose(geometry_msgs::PoseStamped &pose);
    void move_arm_cartesian(tf::Transform trans);
    void set_arm_wrench(tf::Vector3 &force, tf::Vector3 &wrench);
    // afxdTipFrame is the affixedTipFrame;

    typedef boost::shared_ptr<Frame> FramePtr;
    FramePtr originFramePtr, afxdTipFramePtr, eeFramePtr, freeFramePtr;
    Command eeCmd;
    std::vector<FramePtr> frameptrVec;
    std::vector<FramePtr>::iterator frameIter;

};
#endif
