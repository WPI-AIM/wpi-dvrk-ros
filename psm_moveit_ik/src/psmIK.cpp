
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
    ros::init(argc, argv, "psmIK_node");
    ros::NodeHandle node;
    ros::Publisher robot_state_publisher = node.advertise<moveit_msgs::DisplayRobotState>( "/psm/robot_state", 1 );
    robot_model_loader::RobotModelLoader rml = robot_model_loader::RobotModelLoader("robot_description");
    robot_model::RobotModelPtr psm_model = rml.getModel();
    robot_state::RobotStatePtr psm_state (new robot_state::RobotState(psm_model));
//  Available groups are "full_chain","full_arm" and "end_effector". Can use either full_chain or full_arm
//  full_chain does not have the last link(end_effector), while full arm has the last link
    robot_model::JointModelGroup* joint_model_group = psm_model->getJointModelGroup("full_chain");
    std::vector<std::string> link_names =joint_model_group->getLinkModelNames();
    std::vector<std::string> variable_names = joint_model_group->getVariableNames();
    std::vector<double> variable_values;
    random_numbers::RandomNumberGenerator rng;
//    joint_model_group->getVariableRandomPositions(rng,variable_values);
//    psm_state->setJointGroupPositions(joint_model_group,variable_values);
    std::string last_link = link_names.back();
    Eigen::Affine3d end_effector_state = psm_state->getFrameTransform(last_link);
    end_effector_state.translation().z() -= 0.03;

    ROS_INFO_STREAM("Translation: for " << last_link << "\n "<< end_effector_state.translation());
    ROS_INFO_STREAM("Rotation: for " << last_link << "\n "<< end_effector_state.rotation());
    bool found_ik = psm_state->setFromIK(joint_model_group, end_effector_state,10,0.1);
    if (found_ik)
    {
      psm_state->copyJointGroupPositions(joint_model_group, variable_values);
//      for(std::size_t i=0; i < variable_names.size(); ++i)
//      {
//        ROS_INFO("Joint %s: %f", variable_names[i].c_str(), variable_values[i]);
//      }
    }
    else
    {
      ROS_INFO("Did not find IK solution");
    }
    psm_state->setJointGroupPositions(joint_model_group,variable_values);
    Eigen::Affine3d new_end_effector_state = psm_state->getFrameTransform(last_link);

    ROS_INFO_STREAM("New Translation: for " << last_link << "\n "<< new_end_effector_state.translation());
    ROS_INFO_STREAM("New Rotation: for " << last_link << "\n "<< new_end_effector_state.rotation());


    std::vector<std::string> joint_names = joint_model_group->getActiveJointModelNames();
    std::vector<double>      joint_values;
//  We need this loop for getting the joint values of only the active joints from Robot Model
//  The robot model considers non active joints as well for going inverse kinematics and motion
//  planning.
    for (std::size_t i=0; i<joint_names.size(); i++)
    {
        for (std::size_t j=0; j<variable_names.size(); j++)
        {
            if(joint_names.at(i) == variable_names.at(j))
            {
                joint_values.push_back(variable_values[j]);
                break;
            }
        }
    }

    if (joint_names.size() != joint_values.size())
    {
        ROS_ERROR("Unequal size of Active Joint Names and Active Joint Values, check Group Name! Aborting");
        return -1;
    }
//        for (int i=0; i<joint_names.size(); i++)
//        {
//            ROS_INFO("Joint :%s has value :%f",joint_names[i].c_str(),joint_values[i]);
//        }

    moveit_msgs::DisplayRobotState display_msg;
    display_msg.state.joint_state.position = joint_values;
    display_msg.state.joint_state.name = joint_names;
    robot_state_publisher.publish(display_msg);
    printf("Success \n");

return 0;
}
