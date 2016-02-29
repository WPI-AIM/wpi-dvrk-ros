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

#include <mtm_haptics/haptic_position_kinematics.h>


class MTMHaptics
{

public:
    MTMHaptics();

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
    void haptic_feeback_plane(geometry_msgs::Point set_point, bool _collision);
    void set_effort_mode();
    ros::Rate *rate_;




protected:
    ros::NodeHandle node_;
    void jnt_pos_cb(const sensor_msgs::JointStatePtr &msg);
    void jnt_torque_cb(const sensor_msgs::JointStatePtr &msg);
    void crt_pos_cb(const geometry_msgs::PoseConstPtr &msg);
    void cur_robot_state_cb(const std_msgs::StringConstPtr &msg);
    void convert_bodyForcetoSpatialForce(geometry_msgs::WrenchStamped &wrench);
    void visualize_haptic_force(const ros::Publisher &pub);
    void haptic_collision_cb(const geometry_msgs::PoseConstPtr &deflection);


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

MTMHaptics::MTMHaptics(){
    jnt_pos_sub = node_.subscribe("/dvrk_mtm/joint_position_current", 10, &MTMHaptics::jnt_pos_cb, this);
//    jnt_torque_sub = node_.subscribe("/dvrk_mtm/joint_effort_current", 10, &MTMHaptics::jnt_torque_cb, this);
    crt_pos_sub = node_.subscribe("/dvrk_mtm/cartesian_pose_current", 10, &MTMHaptics::crt_pos_cb, this);
    haptic_collision_sub = node_.subscribe("/dvrk_mtm/haptic_collision_subscriber",10, &MTMHaptics::haptic_collision_cb, this);

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

    dX.x = 0.05;
    dX.y = 0.05;
    dX.z = 0.05;

}

void MTMHaptics::jnt_pos_cb(const sensor_msgs::JointStatePtr &msg){
    cur_mtm_jnt_pos = msg->position;
}

void MTMHaptics::jnt_torque_cb(const sensor_msgs::JointStatePtr &msg){
    cur_mtm_effort = msg->effort;
}

void MTMHaptics::crt_pos_cb(const geometry_msgs::PoseConstPtr &msg){
    pre_mtm_pose.position = cur_mtm_pose.position;
    pre_mtm_pose.orientation = cur_mtm_pose.orientation;
    cur_mtm_pose.position = msg->position;
    cur_mtm_pose.orientation = msg->orientation;

    cur_mtm_tip_vel.x = cur_mtm_pose.position.x - pre_mtm_pose.position.x;
    cur_mtm_tip_vel.y = cur_mtm_pose.position.y - pre_mtm_pose.position.y;
    cur_mtm_tip_vel.z = cur_mtm_pose.position.z - pre_mtm_pose.position.z;
}

void MTMHaptics::cur_robot_state_cb(const std_msgs::StringConstPtr &msg){
    cur_robot_state = msg->data;
}

void MTMHaptics::set_effort_mode(){
    robot_state_cmd.data = "DVRK_EFFORT_CARTESIAN";
    robot_state_pub.publish(robot_state_cmd);
    ROS_INFO("Setting Robot state as: %s",robot_state_cmd.data.c_str());
    rate_->sleep();
}

void MTMHaptics::convert_bodyForcetoSpatialForce(geometry_msgs::WrenchStamped &body_wrench){

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

void MTMHaptics::visualize_haptic_force(const ros::Publisher &pub){
    pub.publish(haptic_feedback);
}

void MTMHaptics::haptic_feeback_plane(geometry_msgs::Point set_point, bool _collision){
    if(strcmp(this->cur_robot_state.c_str(),"DVRK_EFFORT_CARTESIAN")){
        geometry_msgs::Point error;
        error.x = set_point.x - cur_mtm_pose.position.x;
        error.y = set_point.y - cur_mtm_pose.position.y;
        error.z = set_point.z - cur_mtm_pose.position.z;

        if (error.y >= 0 && error.y <= dX.y){
            haptic_feedback.header.stamp = ros::Time::now();
            haptic_feedback.wrench.force.x = 0;
            haptic_feedback.wrench.force.y = (Kp.y * error.y);
            haptic_feedback.wrench.force.z = 0;
            convert_bodyForcetoSpatialForce(haptic_feedback);
            crt_torque_pub.publish(haptic_feedback.wrench);
        }
        else{
            haptic_feedback.wrench.force.x = 0;
            haptic_feedback.wrench.force.y = 0;
            haptic_feedback.wrench.force.z = 0;
            //convert_bodyForcetoSpatialForce(haptic_feedback);
            crt_torque_pub.publish(haptic_feedback.wrench);
        }

    }
    else{
        ROS_WARN("MTM is %s state, it needs to be in DVRK_EFFORT_CARTESIAN STATE",this->cur_robot_state.c_str());
    }
}

void MTMHaptics::haptic_collision_cb(const geometry_msgs::PoseConstPtr &deflection){
        haptic_feedback.header.stamp = ros::Time::now();
        haptic_feedback.wrench.force.x = Kp.x;// * deflection->position.x - ( Kd.x * velocity->x ) ;
        haptic_feedback.wrench.force.y = Kp.y;// * deflection->position.y - ( Kd.y * velocity->y ) ;
        haptic_feedback.wrench.force.z = Kp.z;// * deflection->position.z - ( Kd.z * velocity->z ) ;
        convert_bodyForcetoSpatialForce(haptic_feedback);
        crt_torque_pub.publish(haptic_feedback.wrench);

}


int main(int argc, char ** argv){
    ros::init(argc,argv,"mtm_haptics_node");
    MTMHaptics haptics;
    sleep(2.0);
    geometry_msgs::Point haptic_point;
    haptic_point.y = -0.38;
    haptics.set_effort_mode();

    while (ros::ok()){
    haptics.haptic_feeback_plane(haptic_point,true);
    haptics.rate_->sleep();
    ros::spinOnce();
    }
}
