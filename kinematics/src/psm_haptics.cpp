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
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/UInt64.h>
#include <std_msgs/Bool.h>

class HapticsPSM{


public:
    struct CollisionPSM{
        bool _is_coll;
        bool _first_contact; //Flag is released after a collision occurs for the first time with a body, only reset when collision is cleared and then happens again
        tf::Vector3 cur_normal; //Current normal
        tf::Vector3 pre_normal; //The previous normal
        std::vector<tf::Vector3> cur_normal_arr;
        tf::Vector3 locked_position;
        tf::Vector3 cur_position;
        geometry_msgs::Pose locked_pose;
        tf::Vector3 def_along_n; //deflection along normal
        tf::Vector3 def_total;  //total deflection from locked pose
        double epsilon; //Small value to compare the change in direction of the normals

        CollisionPSM():_first_contact(true){}
    }coll_psm;

    HapticsPSM();
    moveit::planning_interface::MoveGroup * group;
    moveit::planning_interface::MoveGroup::Plan plan_srv;
    collision_detection::CollisionRequest coll_req;
    collision_detection::CollisionResult coll_res;
    planning_scene::PlanningScene * psm_planning_scene;
    robot_model_loader::RobotModelLoader * robotModel;
    robot_model::RobotModelPtr kinematic_model;
    visualization_msgs::MarkerArray psm_collision_points;
    moveit_msgs::PlanningScene planning_scene_msg;
    moveit_msgs::PlanningSceneWorld world_msg;
    geometry_msgs::Point32 pc_point;
    geometry_msgs::WrenchStamped spr_haptic_force;
    geometry_msgs::Vector3 haptic_deflection;
    tf::Matrix3x3 rot_mat6wrt0;
    tf::Vector3 tf_vec3;
    std::vector<double> collision_depth;
    ros::Rate *rate_;

    ros::Publisher pose_pub;
    ros::Publisher coll_marker_pub;
    ros::Publisher traj_pc_pub;
    ros::Publisher spr_haptic_pub;
    ros::Publisher haptic_deflection_pub;


    ros::Subscriber planning_scene_msg_sub;

    void publishMarkers(visualization_msgs::MarkerArray& markers);
    void planning_scene_cb(moveit_msgs::PlanningScene scene);
    bool check_collison();
    void compute_total_deflection(tf::Vector3 &delta_v);
    void compute_deflection_along_normal(tf::Vector3 &n, tf::Vector3 &d, tf::Vector3 &d_along_n);
    void adjust_locked_position_along_new_n(tf::Vector3 &d, tf::Vector3&new_n, tf::Vector3 &p);
    void get_current_position(tf::Vector3 &v);
    void lock_current_position_for_proxy(tf::Vector3 &v); //Record the current pose and set v equal to it.
    void compute_deflection_force(geometry_msgs::Wrench &wrench, tf::Vector3 &d_along_n);
    void compute_average_normal(std::vector<tf::Vector3> &v_arr, tf::Vector3 &v); //If we have many normals, compute average normal vector
    void compute_force_in_tip_frame(geometry_msgs::Wrench &wrench);
    void get_collision_normals(const collision_detection::CollisionResult::ContactMap& con, std::vector<tf::Vector3> &n);
    bool has_normal_changed(tf::Vector3 &v1, tf::Vector3 &v2);
    void run_haptic_alg();


protected:
    ros::NodeHandle node_;
};

HapticsPSM::HapticsPSM()
{
    rate_ = new ros::Rate(250);
    group = new moveit::planning_interface::MoveGroup("full_chain");
    robotModel = new robot_model_loader::RobotModelLoader("robot_description");
    kinematic_model = robotModel->getModel();
    psm_planning_scene = new planning_scene::PlanningScene(kinematic_model);
    coll_req.group_name = "end_effector";
    coll_req.contacts = true;
    coll_req.max_contacts = 5;
    coll_req.verbose = false;

    planning_scene_msg_sub = node_.subscribe(
                "/move_group/monitored_planning_scene",10,&HapticsPSM::planning_scene_cb,this);


    pose_pub = node_.advertise<geometry_msgs::PoseStamped>
            ("/mp_psm/cartesian_pose_current",1);
    coll_marker_pub = node_.advertise<visualization_msgs::MarkerArray>
            ("/interactive_robot_array",100);
    traj_pc_pub = node_.advertise<sensor_msgs::PointCloud>
            ("/psm/trajectory_points_pointcloud",1);
    spr_haptic_pub = node_.advertise<geometry_msgs::WrenchStamped>
            ("/dvrk_psm/haptics_feedback_force",1);
    haptic_deflection_pub = node_.advertise<geometry_msgs::Vector3>
            ("/dvrk_psm/haptic_deflection_vector",1);
    spr_haptic_force.header.frame_id = "one_tool_wrist_sca_ee_link_1";

    rot_mat6wrt0.setRPY(-M_PI/2,0,0);

    group->setEndEffectorLink("one_tool_wrist_sca_ee_link_1");
    coll_psm.epsilon = 0.0001;
}


void HapticsPSM::planning_scene_cb(moveit_msgs::PlanningScene scene){
    world_msg = scene.world;
    //ROS_INFO("Processing Planning Scene Msg");
    psm_planning_scene->processPlanningSceneWorldMsg(world_msg);
}

void HapticsPSM::publishMarkers(visualization_msgs::MarkerArray& markers)
 {
   // delete old markers
   if (psm_collision_points.markers.size())
   {
     for (int i=0; i<psm_collision_points.markers.size(); i++)
       psm_collision_points.markers[i].action = visualization_msgs::Marker::DELETE;

     coll_marker_pub.publish(psm_collision_points);
   }

   // move new markers into g_collision_points
   std::swap(psm_collision_points.markers, markers.markers);

   // draw new markers (if there are any)
   if (psm_collision_points.markers.size())
     coll_marker_pub.publish(psm_collision_points);
 }

void HapticsPSM::get_collision_normals(const collision_detection::CollisionResult::ContactMap &con, std::vector<tf::Vector3> &n){
    n.clear();
    std::map<std::string, unsigned> ns_counts;
    tf::Vector3 temp;
   for(collision_detection::CollisionResult::ContactMap::const_iterator it = con.begin(); it != con.end(); ++it)
    {
      for(unsigned int i = 0; i < it->second.size(); ++i)
      {
        std::string ns_name = it->second[i].body_name_1 + "=" + it->second[i].body_name_2;
        if (ns_counts.find(ns_name) == ns_counts.end())
          ns_counts[ns_name] = 0;
        else
        ns_counts[ns_name]++;
        temp.setValue(it->second[i].normal.x(),it->second[i].normal.y(),it->second[i].normal.z());
        n.push_back(temp);
      }
   }
}

bool HapticsPSM::check_collison(){

    coll_res.clear();
    psm_planning_scene->checkCollisionUnpadded(coll_req,coll_res,*group->getCurrentState(),
                                            psm_planning_scene->getAllowedCollisionMatrix());
    if (coll_res.collision)
       {
         //ROS_INFO("COLLIDING contact_point_count=%d",(int)coll_res.contact_count);

         if (coll_res.contact_count > 0)
         {
           std_msgs::ColorRGBA color;
           color.r = 1.0;
           color.g = 0.0;
           color.b = 0.2;
           color.a = 0.8;
           visualization_msgs::MarkerArray markers;
           collision_detection::getCollisionMarkersFromContacts(markers,
                                                                group->getPlanningFrame().c_str(),
                                                                coll_res.contacts,
                                                                color,
                                                                ros::Duration(), // remain until deleted
                                                                0.002);
           publishMarkers(markers);
           }
         return true;
         }
    else
      {
        //ROS_INFO("Not colliding");

        // delete the old collision point markers
        visualization_msgs::MarkerArray empty_marker_array;
        publishMarkers(empty_marker_array);
        return false;
      }

}


void HapticsPSM::get_current_position(tf::Vector3 &v){
    v.setValue(group->getCurrentPose().pose.position.x,
               group->getCurrentPose().pose.position.y,
               group->getCurrentPose().pose.position.z);
}

void HapticsPSM::lock_current_position_for_proxy(tf::Vector3 &v){
    get_current_position(v); //Whatever the current end position is, set v equal to it.
}

void HapticsPSM::compute_total_deflection(tf::Vector3 &delta_v){
    tf::Vector3 v1,v2;
    v1 = coll_psm.locked_position;
    get_current_position(v2);
    delta_v = v1 - v2;
}

void HapticsPSM::compute_average_normal(std::vector<tf::Vector3> &v_arr, tf::Vector3 &v){
    tf::Vector3 new_n;
    new_n.setValue(0,0,0);
    if(v_arr.size() == 1){
        new_n = v_arr.at(0);
    }
    else{
        for(size_t i ; i < v_arr.size() ; i++){
            new_n = new_n + v_arr.at(i);
        }
    }
    new_n.normalize();

    if(v.length() == 0){
        v = new_n; // This is for the first contact, since the current normal is zero, so assign the new value of the normal computed form collision checking
    }
    else{
        //Check if new_n = 0 or in the negative direction as curr n, if it is, leave n unchanged
        if(v.dot(new_n) == 0){ // If the dot product of the last and the current normal in 0, this could be due to the bug of getting negative normals
            //v remains unchanged
            ROS_INFO("New Normal Cancels out the previous one");
        }
        else if(v.dot(new_n) < 0 ){ //If the dot product of the last and the current normal in a negative number, this could be due to the bug of getting negative normals
            v = -new_n;
            ROS_INFO("New normal lies on the opposite plane");
        }
        else{
            v = new_n;
        }
    }
}

bool HapticsPSM::has_normal_changed(tf::Vector3 &v1, tf::Vector3 &v2){
    tf::Vector3 v;
    v = v1 - v2;
    v.normalize();
    if(v.length() > coll_psm.epsilon){
        return true; //The new normal has changed
    }
    else{
        return false; //The new normal has not changed
    }
}

void HapticsPSM::compute_deflection_along_normal(tf::Vector3 &n, tf::Vector3 &d, tf::Vector3 &d_along_n){
    d_along_n = (d.dot(n)/n.length()) * n; //This gives us the dot product of current deflection in the direction of current normal                                                                                        //We don't need to divide by mag to n, as n is a normal vector with mag = 1
}

void HapticsPSM::adjust_locked_position_along_new_n(tf::Vector3 &d, tf::Vector3 &new_n, tf::Vector3 &p){
    get_current_position(p); //Record the current position, this would be inside the collision mesh as the normal just changed while in collision
    p = p+d; //If we are already inside a body, this would take the locked proxy point outside the body along the deflection along normal.
}


void HapticsPSM::compute_deflection_force(geometry_msgs::Wrench &wrench, tf::Vector3 &d_along_n){

    wrench.force.x = d_along_n.getX();
    wrench.force.y = d_along_n.getY();
    wrench.force.z = d_along_n.getZ();
}

void HapticsPSM::compute_force_in_tip_frame(geometry_msgs::Wrench &wrench){
    rot_mat6wrt0.setRPY(group->getCurrentRPY().at(0),
                        group->getCurrentRPY().at(1),
                        group->getCurrentRPY().at(2));
    tf_vec3.setValue(wrench.force.x,wrench.force.y,wrench.force.z);
    tf_vec3 = rot_mat6wrt0.transpose() * tf_vec3;
    wrench.force.x = tf_vec3.getX();
    wrench.force.y = tf_vec3.getY();
    wrench.force.z = tf_vec3.getZ();
}


void HapticsPSM::run_haptic_alg(){

    if(check_collison()){
        if(coll_psm._first_contact){
            ROS_INFO("First contact occured");
            lock_current_position_for_proxy(coll_psm.locked_position);
            coll_psm._first_contact = false;
        }
        //Step 1:
        get_collision_normals(coll_res.contacts, coll_psm.cur_normal_arr);
        //Step 2:
        compute_average_normal(coll_psm.cur_normal_arr, coll_psm.cur_normal);
        //Step 3:
        if(has_normal_changed(coll_psm.cur_normal, coll_psm.pre_normal)){
            ROS_INFO("Normal has Changed");
            adjust_locked_position_along_new_n(coll_psm.def_along_n, coll_psm.cur_normal, coll_psm.locked_position);
        }
        //Step 5:
        compute_total_deflection(coll_psm.def_total);
        //Step 6:
        compute_deflection_along_normal(coll_psm.cur_normal, coll_psm.def_total, coll_psm.def_along_n);
        //Step 7:
        compute_deflection_force(spr_haptic_force.wrench,coll_psm.def_along_n);
        //Step 8:
        compute_force_in_tip_frame(spr_haptic_force.wrench);
        //Step 9:
        coll_psm.pre_normal = coll_psm.cur_normal;

    }
    else{
        if(!coll_psm._first_contact){
            ROS_INFO("First contact Released");
            coll_psm._first_contact = true;
        }
        coll_psm.def_along_n.setValue(0,0,0);
        coll_psm.cur_normal.setValue(0,0,0);
        spr_haptic_force.wrench.force.x=0;
        spr_haptic_force.wrench.force.y=0;
        spr_haptic_force.wrench.force.z=0;
    }
    haptic_deflection.x = coll_psm.def_along_n.getX();
    haptic_deflection.y = coll_psm.def_along_n.getY();
    haptic_deflection.z = coll_psm.def_along_n.getZ();
    haptic_deflection_pub.publish(haptic_deflection);
    spr_haptic_pub.publish(spr_haptic_force);


}

int main(int argc, char ** argv)
{
    ros::init(argc,argv,"psm_haptics_node");
    HapticsPSM psmHap;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    while(ros::ok())
    {
        psmHap.run_haptic_alg();
        psmHap.rate_->sleep();
    }
    ros::shutdown();
    return 0;
}
