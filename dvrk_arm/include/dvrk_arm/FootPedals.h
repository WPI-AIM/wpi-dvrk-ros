#ifndef CDVRK_FOOTPEDALSH
#define CDVRK_FOOTPEDALSH

class DVRK_Bridge;

class DVRK_FootPedals{
  friend class DVRK_Bridge;
  DVRK_FootPedals();
  ~DVRK_FootPedals();
private:
      ros::Subscriber clutch_sub, coag_sub;
      bool _clutch_pressed, _coag_pressed;
      void clutch_sub_cb(const sensor_msgs::JoyConstPtr &msg);
      void coag_sub_cb(const sensor_msgs::JoyConstPtr &msg);
};
#endif
