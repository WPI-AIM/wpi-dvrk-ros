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

class DVRK_Arm{
public:
    DVRK_Arm(const std::string &arm_name);
    ~DVRK_Arm();

    bool home();
    void _rate_sleep();

    bool _is_available();
    bool _in_effort_mode();
    bool _in_cart_pos_mode();
    bool _in_jnt_pos_mode();

    bool set_mode(std::string str);
    bool set_force(const double &fx,const double &fy,const double &fz);
    bool set_moment(const double &nx,const double &ny,const double &nz);
    bool set_wrench(const double &fx,const double &fy,const double &fz,const double &nx,const double &ny,const double &nz);

    void set_origin_pos(const double &x, const double &y, const double &);
    void set_origin_pos(const geometry_msgs::Point &pos);
    void set_origin_pos(const tf::Vector3 &pos);

    void reorient_ee_frame(const double &roll, const double &pitch, const double &yaw);
    void reorient_ee_frame(const tf::Quaternion &tf_quat);
    void reorient_ee_frame(const geometry_msgs::Quaternion &gm_quat);
    void reorient_ee_frame(const tf::Matrix3x3 &mat);

    bool set_position(const double &x, const double &y, const double &z);
    bool set_position(const geometry_msgs::Point &pos);
    bool set_position(const tf::Vector3 &pos);

    bool set_orientation(const double &roll, const double &pitch, const double &yaw);
    bool set_orientation(const tf::Quaternion &tf_quat);
    bool set_orientation(const geometry_msgs::Quaternion &gm_quat);
    bool set_orientation(const tf::Matrix3x3 &mat);

    bool set_pose(const geometry_msgs::PoseStamped &pose);

    void get_cur_position(double &x, double &y, double &z);
    void get_cur_position(tf::Vector3 &pos);
    void get_cur_position(geometry_msgs::Point &pos);

    void get_cur_orientation(double &roll, double &pitch, double &yaw);
    void get_cur_orientation(tf::Quaternion &tf_quat);
    void get_cur_orientation(geometry_msgs::Quaternion &gm_quat);
    void get_cur_orientation(tf::Matrix3x3 &mat);

    void get_cur_pose(geometry_msgs::Pose &pose);

    std::string _m_effort_mode = "DVRK_EFFORT_CARTESIAN";
    std::string _m_jnt_pos_mode = "DVRK_POSITION_JOINT";
    std::string _m_cart_pos_mode = "DVRK_POSITION_CARTESIAN";

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


    geometry_msgs::PoseStamped cur_pose, pre_pose, cmd_pose;
    geometry_msgs::Point origin_pos;
    sensor_msgs::JointState cur_joint, pre_joint;
    std_msgs::String cur_state;
    geometry_msgs::Wrench cur_wrench, cmd_wrench;
    geometry_msgs::Quaternion gm_cur_ori;
    tf::Quaternion tf_cur_ori;
    tf::Matrix3x3 mat_ori, reorient_mat;
    std_msgs::String state_cmd;


};
#endif
