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
//    moveit::planning_interface::MoveGroup group("full_arm");

    ros::NodeHandle node;
    ros::Publisher robot_state_publisher = node.advertise<moveit_msgs::DisplayRobotState>( "/psm/robot_state", 1 );


//    moveit::planning_interface::MoveGroup::Plan my_plan;
    robot_model::RobotModelPtr psm_model = robot_model_loader::RobotModelLoader("robot_description").getModel();
    robot_state::RobotStatePtr psm_state (new robot_state::RobotState(psm_model));

    robot_model::JointModelGroup* joint_model_group = psm_model->getJointModelGroup("full_chain");

//    std::vector<robot_model::JointModel*> joint_model = psm_model->getActiveJointModels();
    std::vector<std::string> link_names =joint_model_group->getLinkModelNames();
    std::vector<std::string> joint_names = joint_model_group->getVariableNames();
    std::vector<double> joint_values;
    random_numbers::RandomNumberGenerator rng;
    joint_model_group->getVariableRandomPositions(rng,joint_values);
   psm_state->setJointGroupPositions(joint_model_group,joint_values);

    Eigen::Affine3d end_effector_state = psm_state->getGlobalLinkTransform(link_names[link_names.size()-1]);
    bool found_ik = psm_state->setFromIK(joint_model_group,end_effector_state,10,0.1);
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
    ROS_INFO_STREAM("Translation: for " << link_names[link_names.size()-1].c_str()<< "\n "<< end_effector_state.translation());
    moveit_msgs::DisplayRobotState display_msg;
//    for(int i=0;i<joint_values.size();i++)
//    {
//        ROS_INFO(" Joints values are %f",joint_values[i]);
//    }
    display_msg.state.joint_state.position = joint_values;
    display_msg.state.joint_state.name = joint_names;
    robot_state_publisher.publish(display_msg);

//    const double * pos = ks->getJointPositions("outer_yaw_joint");
//    ROS_INFO("pos after is %f \n", *pos);
//    bool success = group.plan(my_plan);
    while(1);
     printf("Success \n");

return 0;
}
