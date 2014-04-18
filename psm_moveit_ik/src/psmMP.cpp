//Author : Adnan Munawar
//Email : amunawar@wpi.edu

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <geometry_msgs/Transform.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/PlanningScene.h>
#include <ros/time.h>

class Kinematic_group{

public:
    Kinematic_group();
    moveit::planning_interface::MoveGroup * group;
    ros::Subscriber trajectory_sub;
    void trajectory_cb(const geometry_msgs::PoseConstPtr & pose);
    void generate_trajectory();

protected:
    ros::NodeHandle node;
    std::vector<geometry_msgs::Pose> waypoint;
    moveit_msgs::RobotTrajectory path;
};

Kinematic_group::Kinematic_group(){

    this->group = new moveit::planning_interface::MoveGroup("full_chain");
    this->trajectory_sub = node.subscribe("/mtm/trajectory_poses",1000,&Kinematic_group::trajectory_cb,this);
}

void Kinematic_group::trajectory_cb(const geometry_msgs::PoseConstPtr &pose)
{
    this->waypoint.push_back(*pose.get());
}

void Kinematic_group::generate_trajectory()
{
    double fraction = this->group->computeCartesianPath(
                this->waypoint,0.1,0,this->path);
}

int main(int argc, char ** argv)
{
    ros::init(argc,argv,"psm_MotionPlanning_node");
    ros::NodeHandle node;

    return 0;
}
