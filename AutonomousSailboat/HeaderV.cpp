#include "HeaderV.h"
#include <Sailboat.h>

Header::Header() {
}
void Header::init() {
}
void Header::Control(const geometry_msgs::Twist& cmd) {
  double theta = 0;
  double rudder = 0;
  double sail = 0;
  
  if(cmd.linear.x == 0 && cmd.linear.y == 0){
    theta = cmd.angular.z;
  }else{
    double x = cmd.linear.x;
    double y = cmd.linear.y;

    double norm = sqrt(x * x + y * y);
    theta = atan2(y / norm, x / norm) + M_PI/2.0;
  }

  XSens* xsens = Sailboat::Instance()->getIMU();
  WindSensor* wind = Sailboat::Instance()->getWindSensor();

  float yaw = xsens->getHeadingYaw();

  if (cos(yaw - theta) >= 0)
    rudder = RUDDER_MAX * sin(yaw - theta);
  else
    rudder = RUDDER_MAX * sign(sin(yaw - theta));

  sail = SAIL_MAX * (cos(wind->getMeasure()+yaw-theta) + 1) / 2;

  Sailboat::Instance()->getRudder()->applyCommand(rudder);
  Sailboat::Instance()->getSail()->applyCommand(abs(sail));
#ifdef ACTUATOR_RUDDER2
  Sailboat::Instance()->getRudder2()->applyCommand(rudder/4.5);
#endif
}
