#include "Footpedals.h"

DVRK_FootPedals::DVRK_FootPedals(){
    clutch_sub = n->subscribe("/dvrk/footpedals/clutch", 10, &DVRK_Arm::clutch_sub_cb, this);
    coag_sub = n->subscribe("/dvrk/footpedals/coag", 10, &DVRK_Arm::coag_sub_cb, this);
}

void DVRK_FootPedals::clutch_sub_cb(const sensor_msgs::JoyConstPtr &msg){
    _clutch_pressed = msg->buttons[0];
}

void DVRK_FootPedals::coag_sub_cb(const sensor_msgs::JoyConstPtr &msg){
    _coag_pressed = msg->buttons[0];
}
