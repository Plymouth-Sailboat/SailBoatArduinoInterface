#include "RudderSailControl.h"
#include <Sailboat.h>

RudderSail::RudderSail(){
  
}

void RudderSail::init() {
  
}

void RudderSail::Control(const geometry_msgs::Twist& cmd) {
  double rudder = cmd.angular.x * RAD_TO_DEG;
  double sail = cmd.angular.y * RAD_TO_DEG;
  double rudder2 = cmd.angular.z * RAD_TO_DEG;
  Sailboat::Instance()->getRudder()->applyCommand(rudder);
  Sailboat::Instance()->getSail()->applyCommand(abs(sail));
#ifdef ACTUATOR_RUDDER2
  Sailboat::Instance()->getRudder2()->applyCommand(rudder2);
#endif
}
