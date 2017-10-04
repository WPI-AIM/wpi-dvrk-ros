#ifndef CDVRK_MTMH
#define CDVRK_MTMH

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Wrench.h"
#include "string.h"
#include "tf/tf.h"
#include "tf/LinearMath/Matrix3x3.h"

class DVRK_MTM{
public:
    DVRK_MTM();
    ~DVRK_MTM();

    bool home();

    void state_sub_cb(const std_msgs::StringConstPtr &msg);
    void pose_sub_cb(const geometry_msgs::PoseStampedConstPtr &msg);
    void joint_sub_cb(const sensor_msgs::JointStateConstPtr &msg);
    void init();
    void _rate_sleep();

    bool _is_mtm_available();
    bool _in_effort_mode();
    bool set_mode(std::string str);
    bool set_force(double fx, double fy, double fz);

    geometry_msgs::PoseStamped cur_pose, pre_pose;
    sensor_msgs::JointState cur_joint, pre_joint;
    std_msgs::String cur_state;
    geometry_msgs::Wrench cur_wrench, cmd_wrench;
    geometry_msgs::Quaternion gm_cur_ori;
    tf::Quaternion tf_cur_ori;
    tf::Matrix3x3 mat_ori, ori_corr;
    std_msgs::String state_cmd;

    std::string arm_name;
    std::string _m_effort_mode = "DVRK_EFFORT_CARTESIAN";
    float pos_offset[3];

private:
    ros::NodeHandle *n;
    ros::Publisher force_pub;
    ros::Publisher force_orientation_safety_pub;
    ros::Subscriber pose_sub;
    ros::Subscriber joint_sub;
    ros::Subscriber state_sub;
    ros::Publisher state_pub;
    ros::Rate *rate;

};
#endif
