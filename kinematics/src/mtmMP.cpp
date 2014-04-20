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
#include <std_msgs/UInt64.h>

class Kinematic_group{

public:
    Kinematic_group();
    moveit::planning_interface::MoveGroup * group;
    ros::Subscriber trajectory_sub;
    ros::Subscriber trajectory_size_sub;
    ros::Publisher trajectory_pub;
    bool traj_generate;

    void trajectory_cb(const geometry_msgs::PoseConstPtr & pose);
    void trajectory_size_cb(const std_msgs::UInt64ConstPtr & size);
    void generate_trajectory();
    void publish_trajectory();

protected:
    ros::NodeHandle node;
    std::vector<geometry_msgs::Pose> waypoint;
    moveit_msgs::RobotTrajectory path;
    ros::Rate *rate;
    std_msgs::UInt64 traj_size;
};

Kinematic_group::Kinematic_group()
{
    this->rate = new ros::Rate(500);
    this->traj_generate = false;
    this->group = new moveit::planning_interface::MoveGroup("full_chain");
    this->trajectory_sub = node.subscribe(
                "/mtm/trajectory_poses",1000,&Kinematic_group::trajectory_cb,this);
    this->trajectory_size_sub = node.subscribe(
                "/mtm/trajectory_poses_size",1000,&Kinematic_group::trajectory_size_cb,this);
    this->trajectory_pub = node.advertise<moveit_msgs::RobotTrajectory>(
                "/moveit_mtm/waypoint",1);
}

void Kinematic_group::trajectory_size_cb(const std_msgs::UInt64ConstPtr &size)
{
    this->traj_size.data = size->data;
}

void Kinematic_group::trajectory_cb(const geometry_msgs::PoseConstPtr &pose)
{
    this->waypoint.push_back(*pose.get());
    if (this->waypoint.size() == this->traj_size.data)
    {
        this->traj_generate = true;
    }
}

void Kinematic_group::generate_trajectory()
{
    double fraction = this->group->computeCartesianPath(
                this->waypoint,0.1,0,this->path);

    if(this->path.joint_trajectory.points.size() > 0)
    {
    this->trajectory_pub.publish(this->path);
        ros::spinOnce();
        this->rate->sleep();
    }
    else
    {
        ROS_INFO("Path not computed succesfully, try again");
    }
}

int main(int argc, char ** argv)
{
    ros::init(argc,argv,"mtm_kinematics_node");
    ros::NodeHandle node;
    Kinematic_group traj;
    while(ros::ok())
    {
        if(traj.traj_generate == true)
        {
            traj.generate_trajectory();
            traj.traj_generate = false;
        }
        ros::spinOnce();
    }

    return 0;
}
