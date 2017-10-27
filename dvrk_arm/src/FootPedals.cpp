#include "dvrk_arm/FootPedals.h"

DVRK_FootPedals::DVRK_FootPedals(){
}

void DVRK_FootPedals::init(boost::shared_ptr<ros::NodeHandle> n){
    clutch_sub = n->subscribe("/dvrk/footpedals/clutch", 10, &DVRK_FootPedals::clutch_sub_cb, this);
    coag_sub = n->subscribe("/dvrk/footpedals/coag", 10, &DVRK_FootPedals::coag_sub_cb, this);
    _clutch_pressed = false;
    _coag_pressed = false;
}

void DVRK_FootPedals::clutch_sub_cb(const sensor_msgs::JoyConstPtr &msg){
    _clutch_pressed = msg->buttons[0];
}

void DVRK_FootPedals::coag_sub_cb(const sensor_msgs::JoyConstPtr &msg){
    _coag_pressed = msg->buttons[0];
}

DVRK_FootPedals::~DVRK_FootPedals(){

}
