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
#include<geometry_msgs/Point32.h>
#include<sensor_msgs/PointCloud.h>

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
    ros::Publisher pc_pub;
    ros::Subscriber trajectory_sub;
    ros::Subscriber trajectory_size_sub;
    ros::Publisher trajectory_pub;
    bool traj_generate;

    void trajectory_cb(const geometry_msgs::PoseConstPtr & pose);
    void trajectory_size_cb(const std_msgs::UInt64ConstPtr & size);
    void generate_trajectory();
    void publish_trajectory();
    void publish_trajectory_as_point_cloud();


protected:
    ros::NodeHandle node;
    std::vector<geometry_msgs::Pose> waypoint;
    moveit_msgs::RobotTrajectory path;
    ros::Rate *rate;
    std_msgs::UInt64 traj_size;
    sensor_msgs::PointCloud pc;
    geometry_msgs::Point32 pc_point;
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
    this->pc_pub = node.advertise<sensor_msgs::PointCloud>("/mtm/waypoint_pc",1);
    this->pc.header.frame_id = "right_base";
}

void Kinematic_group::trajectory_size_cb(const std_msgs::UInt64ConstPtr &size)
{
    this->traj_size.data = size->data;
}

void Kinematic_group::trajectory_cb(const geometry_msgs::PoseConstPtr &pose)
{
    this->waypoint.push_back(*pose.get());
    this->pc_point.__connection_header = pose->__connection_header;
    this->pc_point.x = pose->position.x;
    this->pc_point.y = pose->position.y;
    this->pc_point.z = pose->position.z;

    this->pc.points.push_back(this->pc_point);
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

void Kinematic_group::publish_trajectory_as_point_cloud()
{
    this->pc_pub.publish(pc);
}

int main(int argc, char ** argv)
{
    ros::init(argc,argv,"mtm_kinematics_node");
    ros::NodeHandle node;
    Kinematic_group traj;
    while(ros::ok())
    {
        traj.publish_trajectory_as_point_cloud();
        if(traj.traj_generate == true)
        {
            traj.generate_trajectory();
            traj.traj_generate = false;
        }
        ros::spinOnce();
    }

    return 0;
}
