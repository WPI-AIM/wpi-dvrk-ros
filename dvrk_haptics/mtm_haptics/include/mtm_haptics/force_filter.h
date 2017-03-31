#include <iostream>
#include <vector>
#include <geometry_msgs/Vector3.h>


struct ForceData{
    std::vector<geometry_msgs::Vector3> force_array;
    uint _quene_size = 10;

    ForceData(){
        force_array.resize(_quene_size);
    }

};

class ForceFilter: public ForceData{

    ForceFilter(){

    };

    void filter_force(geometry_msgs::Vector3 &vec){
    }

};
