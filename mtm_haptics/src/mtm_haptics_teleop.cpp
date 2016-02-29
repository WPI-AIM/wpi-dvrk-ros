#include<iostream>
#include<stdio.h>
#include<ros/ros.h>
#include<sensor_msgs/JointState.h>
#include<geometry_msgs/Pose.h>
#include<geometry_msgs/Vector3.h>
#include<geometry_msgs/Wrench.h>
#include<geometry_msgs/WrenchStamped.h>
#include<std_msgs/String.h>
#include<tf/LinearMath/Matrix3x3.h>
#include<tf/LinearMath/Vector3.h>



class HapticsMTM
{

    struct CollisionMTM{
        tf::Vector3 def_mtm_base; //deflection in MTM Base

    }coll_mtm;

public:
    HapticsMTM();

    ros::Publisher crt_torque_pub;
    ros::Publisher jnt_torque_pub;
    ros::Publisher robot_state_pub;
    ros::Publisher body_force_pub;  //Force defined in tool frame
    ros::Publisher spatial_force_pub; //Force defined in base frame

    ros::Subscriber jnt_pos_sub;
    ros::Subscriber jnt_torque_sub;
    ros::Subscriber crt_pos_sub;
    ros::Subscriber cur_robot_state_sub;
    ros::Subscriber haptic_collision_sub;
    ros::Subscriber haptic_deflection_sub;
    void haptic_feeback_plane(geometry_msgs::Point set_point, bool _collision);
    void set_effort_mode();
    void run_mtm_haptic_alg();
    ros::Rate *rate_;




protected:
    ros::NodeHandle node_;
    void jnt_pos_cb(const sensor_msgs::JointStatePtr &msg);
    void jnt_torque_cb(const sensor_msgs::JointStatePtr &msg);
    void crt_pos_cb(const geometry_msgs::PoseConstPtr &msg);
    void cur_robot_state_cb(const std_msgs::StringConstPtr &msg);
    void convert_bodyForcetoSpatialForce(geometry_msgs::WrenchStamped &wrench);
    void visualize_haptic_force(const ros::Publisher &pub);
    void haptic_deflection_cb(const geometry_msgs::Vector3ConstPtr &detla_v);


    geometry_msgs::Pose cur_mtm_pose;
    geometry_msgs::Pose pre_mtm_pose;
    geometry_msgs::Vector3 cur_mtm_tip_vel;
    std::vector<double> cur_mtm_effort;
    std::vector<double> cur_mtm_jnt_pos;
    std::string cur_robot_state;
    std_msgs::String robot_state_cmd;
    geometry_msgs::WrenchStamped haptic_feedback;
    geometry_msgs::Vector3 Kp;
    geometry_msgs::Vector3 Kd;
    geometry_msgs::Vector3 dX;
    sensor_msgs::JointState torque_msg;

    tf::Matrix3x3 rot_matrix;
    tf::Vector3 F7wrt0,F0wrt7;
    tf::Quaternion rot_quat;

};

HapticsMTM::HapticsMTM(){
    jnt_pos_sub = node_.subscribe("/dvrk_mtm/joint_position_current", 10, &HapticsMTM::jnt_pos_cb, this);
//  jnt_torque_sub = node_.subscribe("/dvrk_mtm/joint_effort_current", 10, &HapticsMTM::jnt_torque_cb, this);
    crt_pos_sub = node_.subscribe("/dvrk_mtm/cartesian_pose_current", 10, &HapticsMTM::crt_pos_cb, this);
    haptic_deflection_sub = node_.subscribe("/dvrk_psm/haptic_deflection_vector", 10, &HapticsMTM::haptic_deflection_cb, this);

    crt_torque_pub = node_.advertise<geometry_msgs::Wrench>("/dvrk_mtm/set_wrench_body",10);
    jnt_torque_pub = node_.advertise<sensor_msgs::JointState>("/dvrk_mtm/set_effort_joint_unsinked",10);
    robot_state_pub = node_.advertise<std_msgs::String>("/dvrk_mtm/set_robot_state",10);
    body_force_pub = node_.advertise<geometry_msgs::WrenchStamped>("/dvrk_mtm_haptics/body_force_feedback",10);
    spatial_force_pub = node_.advertise<geometry_msgs::WrenchStamped>("/dvrk_mtm_haptics/spatial_force_feedback",10);
    haptic_feedback.header.frame_id = "right_ee_link";

    rate_ = new ros::Rate(500);

    Kp.x = 200;
    Kp.y = 200;
    Kp.z = 200;

    Kd.x = 1;
    Kd.y = 1;
    Kd.z = 1;


}

void HapticsMTM::jnt_pos_cb(const sensor_msgs::JointStatePtr &msg){
    cur_mtm_jnt_pos = msg->position;
}

void HapticsMTM::jnt_torque_cb(const sensor_msgs::JointStatePtr &msg){
    cur_mtm_effort = msg->effort;
}

void HapticsMTM::crt_pos_cb(const geometry_msgs::PoseConstPtr &msg){
    pre_mtm_pose.position = cur_mtm_pose.position;
    pre_mtm_pose.orientation = cur_mtm_pose.orientation;
    cur_mtm_pose.position = msg->position;
    cur_mtm_pose.orientation = msg->orientation;

    cur_mtm_tip_vel.x = cur_mtm_pose.position.x - pre_mtm_pose.position.x;
    cur_mtm_tip_vel.y = cur_mtm_pose.position.y - pre_mtm_pose.position.y;
    cur_mtm_tip_vel.z = cur_mtm_pose.position.z - pre_mtm_pose.position.z;
}

void HapticsMTM::cur_robot_state_cb(const std_msgs::StringConstPtr &msg){
    cur_robot_state = msg->data;
}

void HapticsMTM::set_effort_mode(){
    robot_state_cmd.data = "DVRK_EFFORT_CARTESIAN";
    robot_state_pub.publish(robot_state_cmd);
    ROS_INFO("Setting Robot state as: %s",robot_state_cmd.data.c_str());
    rate_->sleep();
}

void HapticsMTM::convert_bodyForcetoSpatialForce(geometry_msgs::WrenchStamped &body_wrench){

    visualize_haptic_force(body_force_pub);
    rot_quat.setX(cur_mtm_pose.orientation.x);
    rot_quat.setY(cur_mtm_pose.orientation.y);
    rot_quat.setZ(cur_mtm_pose.orientation.z);
    rot_quat.setW(cur_mtm_pose.orientation.w);
    F7wrt0.setValue(body_wrench.wrench.force.x, body_wrench.wrench.force.y, body_wrench.wrench.force.z);
    rot_matrix.setRotation(rot_quat);
    F0wrt7 = rot_matrix.transpose() * F7wrt0;
    body_wrench.wrench.force.x = F0wrt7.x();
    body_wrench.wrench.force.y = F0wrt7.y();
    body_wrench.wrench.force.z = F0wrt7.z();
    visualize_haptic_force(spatial_force_pub);

}


void HapticsMTM::visualize_haptic_force(const ros::Publisher &pub){
    pub.publish(haptic_feedback);
}


void HapticsMTM::haptic_deflection_cb(const geometry_msgs::Vector3ConstPtr &delta_v){
     coll_mtm.def_mtm_base.setValue(delta_v->x,delta_v->y,delta_v->z);
}

void HapticsMTM::run_mtm_haptic_alg(){

    if(strcmp(cur_robot_state.c_str(), "DVRK_EFFOR_CARTESIAN")){
        haptic_feedback.wrench.force.x = Kp.x * coll_mtm.def_mtm_base.getX();
        haptic_feedback.wrench.force.y = Kp.y * coll_mtm.def_mtm_base.getY();
        haptic_feedback.wrench.force.z = Kp.z * coll_mtm.def_mtm_base.getZ();

        convert_bodyForcetoSpatialForce(haptic_feedback);
        crt_torque_pub.publish(haptic_feedback.wrench);
    }
    else{
        set_effort_mode();
        ROS_INFO("Setting Robot State to Cartesian Effort. Current State is %s",cur_robot_state.c_str());
        sleep(3.0);
    }

}


int main(int argc, char ** argv){
    ros::init(argc,argv,"mtm_haptics_teleop_node");
    HapticsMTM haptics;
    sleep(2.0);
    while (ros::ok()){
    haptics.run_mtm_haptic_alg();
    haptics.rate_->sleep();
    ros::spinOnce();
    }
    ros::shutdown();
    return 0;
}
