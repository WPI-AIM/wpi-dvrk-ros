//Author : Adnan Munawar
//Email : amunawar@wpi.edu

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <geometry_msgs/Transform.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/collision_detection_fcl/collision_world_fcl.h>
#include <moveit/collision_detection_fcl/collision_robot_fcl.h>
#include <moveit/collision_detection/collision_tools.h>
#include <ros/time.h>
#include <std_msgs/UInt64.h>
#include <std_msgs/Bool.h>

class Kinematic_group{

public:
    Kinematic_group();
    moveit::planning_interface::MoveGroup * group;
    moveit::planning_interface::MoveGroup::Plan plan_srv;
    std::vector<geometry_msgs::Pose> poses_list;
    moveit_msgs::RobotTrajectory trajectory;
    collision_detection::CollisionRequest coll_req;
    collision_detection::CollisionResult coll_res;
    planning_scene::PlanningScene * g_planning_scene;
    ros::Rate *rate_;

    ros::Publisher pose_pub;
    ros::Publisher coll_marker_pub;

    ros::Subscriber coag_sub;
    ros::Subscriber clutch_sub;

    void coag_cb(const std_msgs::BoolConstPtr &state);
    void clutch_cb(const std_msgs::BoolConstPtr &state);
    void compute_cart_path();


protected:
    ros::NodeHandle node_;
    moveit_msgs::RobotTrajectory path_;
    std_msgs::UInt64 traj_size_;
};

Kinematic_group::Kinematic_group()
{
    this->rate_ = new ros::Rate(50);
    this->group = new moveit::planning_interface::MoveGroup("full_chain");
    this->g_planning_scene = new planning_scene::PlanningScene();
    this->coll_req.group_name = this->group->getName();
    this->coll_req.contacts = true;
    this->coll_req.max_contacts = 100;
    this->coll_req.verbose = false;


    this->coag_sub = node_.subscribe("/dvrk_footpedal/coag_state",10,
                                     &Kinematic_group::coag_cb, this);
    this->clutch_sub = node_.subscribe("/dvrk_footpedal/clutch_state",10,
                                       &Kinematic_group::clutch_cb, this);

    this->pose_pub = node_.advertise<geometry_msgs::PoseStamped>
            ("/mp_psm/cartesian_pose_current",1);
    this->coll_marker_pub = node_.advertise<visualization_msgs::MarkerArray>
            ("interactive_robot_marray",100);
}

void Kinematic_group::coag_cb(const std_msgs::BoolConstPtr & state)
{
    if(state->data == true){
        this->poses_list.push_back(this->group->getCurrentPose().pose);
    }

}


void Kinematic_group::clutch_cb(const std_msgs::BoolConstPtr & state)
{
    if(state->data == true){
        this->compute_cart_path();
    }
}

void Kinematic_group::compute_cart_path(){

    double fraction = group->computeCartesianPath(this->poses_list,
                                                 0.01,  // eef_step
                                                 0.0,   // jump_threshold
                                                 this->trajectory);

    ROS_INFO("Cartesian path is (%.2f%% acheived)",
          fraction * 100.0);
    /* Sleep to give Rviz time to visualize the plan. */
    sleep(5.0);

}

int main(int argc, char ** argv)
{
    ros::init(argc,argv,"psm_MotionPlanning");
    Kinematic_group traj;
    geometry_msgs::PoseStamped pose;
    while(ros::ok())
    {
        pose = traj.group->getCurrentPose();
        traj.pose_pub.publish(pose);
        traj.group->setStartStateToCurrentState();
        ros::spinOnce();
        traj.rate_->sleep();
    }

    return 0;
}
