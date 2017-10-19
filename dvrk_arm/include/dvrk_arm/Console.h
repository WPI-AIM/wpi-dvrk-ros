#ifndef CDVRK_CONSOLEH
#define CDVRK_CONSOLEH
#include "dvrk_arm/Bridge.h"

class DVRK_Bridge;

class DVRK_Console{
  friend class DVRK_Bridge;
  DVRK_Console();
 ~DVRK_Console();
 void init(ros::NodeHandle *n);
private:

};
  #endif
