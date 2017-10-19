#ifndef CDVRK_FOOTPEDALSH
#define CDVRK_FOOTPEDALSH
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"

class DVRK_Bridge;

class DVRK_FootPedals{
  friend class DVRK_Bridge;
  friend class DVRK_Arm;
  DVRK_FootPedals();
  ~DVRK_FootPedals();
  void init(ros::NodeHandle *n);
        bool _clutch_pressed, _coag_pressed;
private:
      ros::Subscriber clutch_sub, coag_sub;
      void clutch_sub_cb(const sensor_msgs::JoyConstPtr &msg);
      void coag_sub_cb(const sensor_msgs::JoyConstPtr &msg);
};
#endif
