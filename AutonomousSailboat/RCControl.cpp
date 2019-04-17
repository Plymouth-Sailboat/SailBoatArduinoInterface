#include "RCControl.h"
#include <Sailboat.h>

RCControl::RCControl(){
  
}

void RCControl::init() {
  
}

void RCControl::Control(const geometry_msgs::Twist& cmd) {
  float sail = Sailboat::Instance()->getRC()->getValue(RC_SAIL);
  float rudder = Sailboat::Instance()->getRC()->getValue(RC_RUDDER);
  float autoCh = Sailboat::Instance()->getRC()->getValue(RC_AUTO);
  Sailboat::Instance()->getRudder()->applyCommand(-(rudder-0.5)*2.0*RUDDER_MAX);
  if(autoCh < 0.5){
    Sailboat::Instance()->getSail()->applyCommand((sail-0.5)*2.0*SAIL_MAX);
  }else{
    WindSensor* wind = Sailboat::Instance()->getWindSensor();
    float sail = SAIL_MAX * (cos(wind->getMeasure()) + 1) / 2;
    Sailboat::Instance()->getSail()->applyCommand(abs(sail));
  }
#ifdef ACTUATOR_RUDDER2
  Sailboat::Instance()->getRudder2()->applyCommand((rudder-0.5)*2.0*RUDDER_MAX/4.5);
#endif
  Sailboat::Instance()->resetWatchdogROS();
}
