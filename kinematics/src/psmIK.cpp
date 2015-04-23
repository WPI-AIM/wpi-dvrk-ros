
//Author : Adnan Munawar
//Email : amunawar@wpi.edu
//College : Worcester Polytechnic Institute


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

class PSMIK{
public:

    PSMIK();


    robot_model::RobotModelPtr psm_model;
    robot_model::RobotStatePtr psm_state;
    robot_model::JointModelGroup* jnt_grp;

protected:

    ros::NodeHandle node_;
    ros::Publisher rob_state_pub_;
    robot_model_loader::RobotModelLoader rob_mod_ldr_;


};


int main(int argc, char ** argv)
{

return 0;
}
