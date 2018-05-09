#include "HeaderV.h"
#include <Sailboat.h>

Header::Header() {
}
void Header::init() {
}
void Header::Control(const geometry_msgs::Twist& cmd) {
  double x = cmd.linear.x;
  double y = cmd.linear.y;

  double norm = sqrt(x * x + y * y);
  double theta = atan2(y / norm, x / norm);

  double rudder = 0;
  double sail = 0;

  XSens* xsens = Sailboat::Instance()->getIMU();
  WindSensor* wind = Sailboat::Instance()->getWindSensor();

  if (cos(xsens->getHeadingYaw() - theta) >= 0)
    rudder = RUDDER_MAX * sin(xsens->getHeadingYaw() - theta);
  else
    rudder = RUDDER_MAX * sign(sin(xsens->getHeadingYaw() - theta));

  sail = HALF_PI * RAD_TO_DEG * (cos(wind->getMeasure()) + 1) / 2;

  Sailboat::Instance()->getRudder()->applyCommand(rudder);
  Sailboat::Instance()->getSail()->applyCommand(sail);
}
