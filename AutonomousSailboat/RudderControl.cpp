#include "RudderControl.h"
#include <Sailboat.h>

RudderControl::RudderControl(){
  
}

void RudderControl::init() {
  
}

void RudderControl::Control(const geometry_msgs::Twist& cmd) {
  Sailboat::Instance()->getRudder()->applyCommand(cmd.angular.x);
#ifdef ACTUATOR_RUDDER2
  Sailboat::Instance()->getRudder2()->applyCommand(cmd.angular.z);
#endif
  double sail = 0;
  WindSensor* wind = Sailboat::Instance()->getWindSensor();

  sail = HALF_PI * RAD_TO_DEG * (cos(wind->getMeasure()) + 1) / 2;
  Sailboat::Instance()->getSail()->applyCommand(sail);
}
