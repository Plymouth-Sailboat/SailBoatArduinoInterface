#include "SailCapControl.h"
#include <Sailboat.h>

SailCap::SailCap(){
  
}

void SailCap::init() {
  
}

void SailCap::Control(const geometry_msgs::Twist& cmd) {
  double x = cmd.linear.x;
  double y = cmd.linear.y;
  double sail = cmd.angular.y * RAD_TO_DEG;

  double norm = sqrt(x * x + y * y);
  double theta = atan2(y / norm, x / norm);
  
  double rudder = 0;

  XSens* xsens = Sailboat::Instance()->getIMU();
  WindSensor* wind = Sailboat::Instance()->getWindSensor();

  float yaw = xsens->getHeadingYaw();

  if (cos(yaw - theta) >= 0)
    rudder = RUDDER_MAX * sin(yaw - theta);
  else
    rudder = RUDDER_MAX * sign(sin(yaw - theta));

  Sailboat::Instance()->getRudder()->applyCommand(rudder);
  Sailboat::Instance()->getSail()->applyCommand(abs(sail));
#ifdef ACTUATOR_RUDDER2
  Sailboat::Instance()->getRudder2()->applyCommand(rudder/4.5);
#endif
}
