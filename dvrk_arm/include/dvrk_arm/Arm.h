#ifndef CDVRK_ArmH
#define CDVRK_ArmH

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Wrench.h"
#include "string.h"
#include "tf/tf.h"
#include "tf/LinearMath/Matrix3x3.h"

struct OriginTrans{
public:
    tf::Transform origin_trans;
    tf::Vector3 origin_pos;
    tf::Quaternion origin_ori_quat;
    tf::Matrix3x3 origin_ori_mat;
};

struct EETrans{
public:
    tf::Transform ee_trans;
    tf::Vector3 ee_pos;
    tf::Quaternion ee_ori_quat;
    tf::Matrix3x3 ee_ori_mat;
};

struct TipTrans{
public:
    tf::Transform tip_trans;
    tf::Transform tip_pos;
    tf::Transform tip_ori_quat;
    tf::Transform tip_ori_mat;
};

struct EETransCmd{
public:
    tf::Transform ee_trans_cmd;
    tf::Vector3 ee_pos_cmd;
    tf::Quaternion ee_ori_quat_cmd;
    tf::Matrix3x3 ee_ori_mat_cmd;
    tf::Vector3 ee_force_cmd;
    tf::Vector3 ee_moment_cmd;
};

class DVRK_Arm: public OriginTrans, public EETrans, public EETransCmd, public TipTrans{
public:
    DVRK_Arm(const std::string &arm_name);
    ~DVRK_Arm();

    bool home();
    void _rate_sleep();

    bool _is_available();
    bool _in_effort_mode();
    bool _in_cart_pos_mode();
    bool _in_jnt_pos_mode();


    void set_origin_pos(const double &x, const double &y, const double &);
    void set_origin_pos(const geometry_msgs::Point &pos);
    void set_origin_pos(const tf::Vector3 &pos);

    void set_origin_rot(const double &roll, const double &pitch, const double &yaw);
    void set_origin_rot(const tf::Quaternion &tf_quat);
    void set_origin_rot(const geometry_msgs::Quaternion &gm_quat);
    void set_origin_rot(const tf::Matrix3x3 &mat);

    void set_origin_trans(const tf::Vector3 &pos, const tf::Quaternion &tf_quat);
    void set_origin_trans(const tf::Vector3 &pos, const tf::Matrix3x3 &tf_mat);
    void set_origin_trans(const tf::Transform &trans);

    void affix_tip_frame(const tf::Transform &trans);

    bool set_mode(std::string str);
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

    const std::string _m_effort_mode = "DVRK_EFFORT_CARTESIAN";
    const std::string _m_jnt_pos_mode = "DVRK_POSITION_JOINT";
    const std::string _m_cart_pos_mode = "DVRK_POSITION_CARTESIAN";

private:
    std::string arm_name;

    ros::NodeHandle *n;
    ros::Publisher force_pub;
    ros::Publisher force_orientation_safety_pub;
    ros::Publisher state_pub;
    ros::Publisher pose_pub;
    ros::Publisher joint_pub;

    ros::Subscriber pose_sub;
    ros::Subscriber joint_sub;
    ros::Subscriber state_sub;
    ros::Subscriber clutch_sub, coag_sub;
    ros::Rate *rate;
    bool _clutch_pressed, _coag_pressed;
    double scale;
    std::vector<std::string> valid_arms;

    void init();
    void state_sub_cb(const std_msgs::StringConstPtr &msg);
    void pose_sub_cb(const geometry_msgs::PoseStampedConstPtr &msg);
    void joint_sub_cb(const sensor_msgs::JointStateConstPtr &msg);
    void clutch_sub_cb(const sensor_msgs::JoyConstPtr &msg);
    void coag_sub_cb(const sensor_msgs::JoyConstPtr &msg);
    void cisstPose_to_userTransform(const geometry_msgs::PoseStamped &pose);
    void userPose_to_cisstPose(geometry_msgs::PoseStamped &pose);
    void move_arm_cartesian(tf::Transform &trans);
    void set_arm_wrench(tf::Vector3 &force, tf::Vector3 &wrench);

    geometry_msgs::PoseStamped cur_pose, pre_pose, cmd_pose;
    sensor_msgs::JointState cur_joint, pre_joint;
    std_msgs::String cur_state;
    geometry_msgs::Wrench cur_wrench, cmd_wrench;
    geometry_msgs::Quaternion gm_cur_ori;
    tf::Quaternion tf_cur_ori;
    tf::Matrix3x3 mat_ori, reorient_mat;
    std_msgs::String state_cmd;
    //ros::AsyncSpinner *aspin;


};
#endif
