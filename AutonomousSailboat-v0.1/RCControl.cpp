#include "RCControl.h"
#include <Sailboat.h>

RCControl::RCControl(){
  
}

void RCControl::init() {
  
}

void RCControl::Control(const geometry_msgs::Twist& cmd) {
  float sail = Sailboat::Instance()->getRC()->getValue(RC_1);
  float rudder = Sailboat::Instance()->getRC()->getValue(RC_3);
  float ch5 = Sailboat::Instance()->getRC()->getRawValue(RC_5);

  Sailboat::Instance()->getRudder()->applyCommand(rudder*RUDDER_MAX);
  Sailboat::Instance()->getSail()->applyCommand(sail*SAIL_MAX);
}
