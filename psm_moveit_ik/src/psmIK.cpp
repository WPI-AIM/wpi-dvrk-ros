
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


int main(int argc, char ** argv)
{
    ros::init(argc, argv, "temp_node");
    ros::NodeHandle node;
    ros::Publisher robot_state_publisher = node.advertise<moveit_msgs::DisplayRobotState>( "/psm/robot_state", 1 );
    robot_model_loader::RobotModelLoader rml = robot_model_loader::RobotModelLoader("robot_description");
    robot_model::RobotModelPtr psm_model = rml.getModel();
    robot_state::RobotStatePtr psm_state (new robot_state::RobotState(psm_model));
    robot_model::JointModelGroup* joint_model_group = psm_model->getJointModelGroup("full_arm");
    std::vector<std::string> link_names =joint_model_group->getLinkModelNames();
    std::vector<std::string> joint_names = joint_model_group->getVariableNames();
        for(int i=0;i<link_names.size();i++)
        {
            ROS_INFO(" Link names are %s",link_names[i].c_str());
        }
        for(int i=0;i<joint_names.size();i++)
        {
            ROS_INFO(" Joint names are %s",joint_names[i].c_str());
        }
    std::vector<double> joint_values;
    random_numbers::RandomNumberGenerator rng;
    joint_model_group->getVariableRandomPositions(rng,joint_values);
   psm_state->setJointGroupPositions(joint_model_group,joint_values);
   Eigen::Affine3d end_effector_state = psm_state->getFrameTransform(link_names[link_names.size()-1].c_str());
   ROS_INFO_STREAM("Translation: for " << link_names[link_names.size()-1].c_str()<< "\n "<< end_effector_state.translation());
//   const kinematics::KinematicsBaseConstPtr& solver = joint_model_group->getSolverInstance();
//   ROS_INFO("Printing this");
//   std::string tip_name = solver->getTipFrame();
//   ROS_INFO(" Tip names is  %s",tip_name.c_str());
   bool found_ik = psm_state->setFromIK(joint_model_group, end_effector_state,3,0.1);
    if (found_ik)
    {
      psm_state->copyJointGroupPositions(joint_model_group, joint_values);
      for(std::size_t i=0; i < joint_names.size(); ++i)
      {
        ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
      }
    }
    else
    {
      ROS_INFO("Did not find IK solution");
    }
    moveit_msgs::DisplayRobotState display_msg;
    display_msg.state.joint_state.position = joint_values;
    display_msg.state.joint_state.name = joint_names;
    robot_state_publisher.publish(display_msg);
    printf("Success \n");

return 0;
}
