#include "RudderSailControl.h"
#include <Sailboat.h>

RudderSail::RudderSail(){
  
}

void RudderSail::init() {
  
}

void RudderSail::Control(const geometry_msgs::Twist& cmd) {
  double rudder = cmd.angular.x * DEG_TO_RAD;
  double sail = cmd.angular.y * DEG_TO_RAD;
  double rudder2 = cmd.angular.z * DEG_TO_RAD;
  Sailboat::Instance()->getRudder()->applyCommand(rudder);
  Sailboat::Instance()->getSail()->applyCommand(sail);
#ifdef ACTUATOR_RUDDER2
  Sailboat::Instance()->getRudder2()->applyCommand(rudder2);
#endif
}
