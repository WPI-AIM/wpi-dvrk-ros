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

/* This class implements the callback functions/subscribers to the dvrk_trajectory.cpp/node file
  The class listens to the vector of geometry poses and stores them in the waypoint variable.
  On the signal to generate a way point, the class parses the poses to generate a waypoint with
  a given step size and jump size.
  To Do: More Description
*/
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
    void plan_path();


protected:
    ros::NodeHandle node_;
    std::vector<geometry_msgs::Pose> waypoint_;
    moveit_msgs::RobotTrajectory path_;
    ros::Rate *rate_;
    std_msgs::UInt64 traj_size_;
};

Kinematic_group::Kinematic_group()
{
    this->rate_ = new ros::Rate(500);
    this->traj_generate = false;
    this->group = new moveit::planning_interface::MoveGroup("full_chain");
    this->group->setPoseReferenceFrame("right_base");
    this->trajectory_sub = node_.subscribe(
                "/mtm/trajectory_poses",1000,&Kinematic_group::trajectory_cb,this);
    this->trajectory_size_sub = node_.subscribe(
                "/mtm/trajectory_poses_size",1000,&Kinematic_group::trajectory_size_cb,this);
    this->trajectory_pub = node_.advertise<moveit_msgs::RobotTrajectory>(
                "/moveit_mtm/waypoint",1);
}

void Kinematic_group::trajectory_size_cb(const std_msgs::UInt64ConstPtr &size)
{
    this->traj_size_.data = size->data;
}

void Kinematic_group::trajectory_cb(const geometry_msgs::PoseConstPtr &pose)
{
    this->waypoint_.push_back(*pose.get());
    if (this->waypoint_.size() == this->traj_size_.data)
    {
        this->traj_generate = true;
    }
}

void Kinematic_group::generate_trajectory()
{
    double fraction = this->group->computeCartesianPath(
                this->waypoint_,0.1,0,this->path_);

    if(this->path_.joint_trajectory.points.size() > 0)
    {
    this->trajectory_pub.publish(this->path_);
        ros::spinOnce();
        this->rate_->sleep();
    }
    else
    {
        ROS_INFO("Path not computed succesfully, try again");
    }
}

void Kinematic_group::plan_path()
{

}


int main(int argc, char ** argv)
{
    ros::init(argc,argv,"mtm_kinematics_node");
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
