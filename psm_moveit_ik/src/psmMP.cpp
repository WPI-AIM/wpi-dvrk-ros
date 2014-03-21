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

int main(int argc, char ** argv)
{
    ros::init(argc,argv,"psm_MotionPlanning_node");
    ros::NodeHandle node;
    moveit::planning_interface::MoveGroup group("full_chain");
    moveit::planning_interface::PlanningSceneInterface scene_interface;
    ros::Publisher display_publisher = node.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;
    ros::Publisher planning_scene_diff_publisher = node.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    while(planning_scene_diff_publisher.getNumSubscribers() < 1)
    {
      ros::WallDuration sleep_t(0.5);
      sleep_t.sleep();
    }
    ROS_INFO("Entering Collision Placing Code");
    moveit_msgs::CollisionObject collision_object;
    collision_object.id = "box";

    shape_msgs::SolidPrimitive prim;
    prim.type = prim.BOX;
    prim.dimensions.resize(3);
    prim.dimensions[0] = 0.05;
    prim.dimensions[1] = 0.05;
    prim.dimensions[2] = 0.05;

    collision_object.primitives.push_back(prim);

    geometry_msgs::Pose prim_pose;
    prim_pose.orientation.w = 1.0;
    prim_pose.position.x = -0.22;
    prim_pose.position.y = -0.45;
    prim_pose.position.z = 0.43;

    collision_object.primitive_poses.push_back(prim_pose);

    ROS_INFO("Adding the object into the world at the location of the right wrist.");
    moveit_msgs::PlanningScene planning_scene;
    planning_scene.world.collision_objects.push_back(collision_object);
    planning_scene.is_diff = true;
    planning_scene_diff_publisher.publish(planning_scene);
    ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
    ROS_INFO("End Effector name is %s ", group.getEndEffectorLink().c_str());

    geometry_msgs::PoseStamped pose_stmp = group.getCurrentPose();
    geometry_msgs::Pose pose = pose_stmp.pose;
    ROS_INFO("X : %f Y: %f Z: %f",pose.position.x,pose.position.y,pose.position.z);
     ROS_INFO("roll : %f pitch: %f yaw: %f w: %f",pose.orientation.x,pose.orientation.y,
              pose.orientation.z,pose.orientation.w);
     moveit::planning_interface::MoveGroup::Plan my_plan;

    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(pose);
    geometry_msgs::Pose target_pose = pose;
    target_pose.position.y += 0.0;
    target_pose.position.z -= 0.1;
    waypoints.push_back(target_pose);  // up and out
    target_pose.position.y -= 0.05;
    waypoints.push_back(target_pose);  // up and out
    target_pose.position.x -= 0.05;
    waypoints.push_back(target_pose);  // up and out
    target_pose.position.y += 0.05;
    waypoints.push_back(target_pose);  // up and out
    target_pose.position.x += 0.05;
    waypoints.push_back(target_pose);  // left
    target_pose.position.z +=0.1;
    waypoints.push_back(target_pose);  // down and right (back to start)
    moveit_msgs::RobotTrajectory trajectory;
    double fraction = group.computeCartesianPath(waypoints,0.01,0.0,trajectory,true);
    display_trajectory.trajectory.push_back(trajectory);
    display_publisher.publish(display_trajectory);
    return 0;
}
