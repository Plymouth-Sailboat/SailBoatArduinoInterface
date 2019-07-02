#include "RudderControl.h"
#include <Sailboat.h>

RudderControl::RudderControl(){
  
}

void RudderControl::init() {
  
}

void RudderControl::Control(const geometry_msgs::Twist& cmd) {
  double rudder = cmd.angular.x * RAD_TO_DEG;
  double rudder2 = cmd.angular.z * RAD_TO_DEG;

  IMU *imu = Sailboat::Instance()->getIMU();
  
  Sailboat::Instance()->getRudder()->applyCommand(rudder);
#ifdef ACTUATOR_RUDDER2
  Sailboat::Instance()->getRudder2()->applyCommand(rudder2);
#endif
  double sail = 0;
  WindSensor* wind = Sailboat::Instance()->getWindSensor();

  double theta = imu->getHeading();
  double cap = theta-asin(cmd.angular.x/(RUDDER_MAX*DEG_TO_RAD));

  sail = SAIL_MAX * (cos(wind->getMeasure() - cap) + 1) / 2;
  Sailboat::Instance()->getSail()->applyCommand(abs(sail));
}
