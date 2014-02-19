#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/tree.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/frames_io.hpp>
#include <stdio.h>
#include <iostream>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>

using namespace KDL;
using namespace std;
class DaVinci_IK
{
public:
    KDL::Chain my_kdl_chain;
    KDL::Tree my_kdl_tree;
    urdf::Model my_urdf_model;
    KDL::JntArray joint_kdl_positions;
    unsigned int nrJnts;
    bool read_urdf(std::string);
    void calculate_FK();
    void calculate_IK();
    void set_IK_solver();
    void auto_set_everything();
    void get_joint_angles();


};


bool DaVinci_IK::read_urdf(std::string file_path)
{
     //Building URDF model from a give file name in xml/urdf format.
        std::string urdf_name = file_path;
        if (!this>my_urdf_model.initFile(urdf_name)){
             ROS_ERROR("Failed to parse urdf robot model");
             return false;
          }
        else {
            printf("Succesfully parsed Robot Description file \n");
        }

        //Building KDL Tree from the URDF model constructed above.
        if (!kdl_parser::treeFromUrdfModel(this->my_urdf_model, this->my_kdl_tree)){
             ROS_ERROR("Failed to construct kdl tree");
             return false;
          }
        else {
            printf("Succesfully constructed KDL tree for the Robot Description file \n");
        }

            bool status =   this->my_kdl_tree.getChain("world","left_arm_outer_pitch_top_link",this->my_kdl_chain);
           if(status)
               printf("Successfully extracted chain from Tree\n");
           else
               printf("Failure in Chain Extraction from Tree\n");

           this->nrJnts = this->my_kdl_chain.getNrOfJoints();
           this->joint_kdl_positions = KDL::JntArray(nrJnts);




      return true;
}

void DaVinci_IK::calculate_FK()
{
        KDL::ChainFkSolverPos_recursive my_FK_solver(this->my_kdl_chain);
        for(unsigned int i=0; i<this->nrJnts ; i++)
        {
            float myinput;
            printf("Enter the position of the joint %i: ",i);
            scanf("%e", &myinput);
           this-> joint_kdl_positions(i)=(double)myinput;
        }
        KDL::Frame cartpos;
        bool kinematic_status;
        kinematic_status = my_FK_solver.JntToCart(this->joint_kdl_positions,cartpos);
        if(kinematic_status>=0){
                std::cout << cartpos <<std::endl;
                printf("%s \n","Found the Forward Kinematics Solutions!");
            }else{
                printf("%s \n","Error: could not calculate forward kinematics :(");
           }
}

void DaVinci_IK::get_joint_angles()
{
    for(unsigned int i=0; i<this->nrJnts ; i++)
    {
        float myinput;
        printf("Enter the position of the joint %i: ",i);
        scanf("%e", &myinput);
       this-> joint_kdl_positions(i)=(double)myinput;
    }
}

void DaVinci_IK::calculate_IK()
{
        KDL::Chain chain;
        chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(0.0,0.0,1.020))));
        chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.0,0.0,0.480))));
        chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.0,0.0,0.645))));
        chain.addSegment(Segment(Joint(Joint::RotZ)));
        chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.0,0.0,0.120))));
        chain.addSegment(Segment(Joint(Joint::RotZ)));

        // Create solver based on kinematic chain
        ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain);

        // Create joint array
        unsigned int nj = chain.getNrOfJoints();
        KDL::JntArray jointpositions = JntArray(nj);

        // Assign some values to the joint positions
        for(unsigned int i=0;i<nj;i++){
            float myinput;
            printf ("Enter the position of joint %i: ",i);
            scanf ("%e",&myinput);
            jointpositions(i)=(double)myinput;
        }

        // Create the frame that will contain the results
        KDL::Frame cartpos;

        // Calculate forward position kinematics
        bool kinematics_status;
        kinematics_status = fksolver.JntToCart(jointpositions,cartpos);
        if(kinematics_status>=0){
            std::cout << cartpos <<std::endl;
            printf("%s \n","Succes, thanks KDL!");
        }else{
            printf("%s \n","Error: could not calculate forward kinematics :(");
        }

}
