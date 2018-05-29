#include "ReturnHome.h"
#include <Sailboat.h>

ReturnHome::ReturnHome(){
}
void ReturnHome::init() {
}
void ReturnHome::Control(const geometry_msgs::Twist& cmd) {
  GPS* gps = Sailboat::Instance()->getGPS();
  double posInit[2] = {gps->getLatInit(), gps->getLongInit()};
  double posActual[2] = {gps->getLat(), gps->getLong()};
  
  const long earth_r = 6371000;

  double longDif = (posInit[1] - posActual[1])*M_PI/180.0;
  double latDif = (posInit[0] - posActual[0])*M_PI/180.0;

  double lat1 = posActual[0]*M_PI/180.0;
  double lat2 = posInit[0]*M_PI/180.0;
  double long1 = posActual[1]*M_PI/180.0;
  double long2 = posInit[1]*M_PI/180.0;

  
  double a = sin(latDif/2.0)*sin(latDif/2.0) +
    cos(lat1) * cos(lat2) *
    sin(longDif/2.0) * sin(longDif/2.0);
  double c = 2 * atan2(sqrt(a), sqrt(1-a));
  double d = earth_r*c;


  double xx = cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(long2-long1);
  double yy = sin(long2-long1) * cos(lat2);
  double bearing = atan2(yy,xx);

  double rudder = 0;
  double sail = 0;

  XSens* xsens = Sailboat::Instance()->getIMU();
  WindSensor* wind = Sailboat::Instance()->getWindSensor();

  float yaw = xsens->getHeadingYaw();

  if (cos(yaw - bearing) >= 0)
    rudder = RUDDER_MAX * sin(yaw - bearing);
  else
    rudder = RUDDER_MAX * sign(sin(yaw - bearing));

  sail = HALF_PI * RAD_TO_DEG * (cos(wind->getMeasure()) + 1) / 2;

  Sailboat::Instance()->getRudder()->applyCommand(rudder);
  Sailboat::Instance()->getSail()->applyCommand(sail);
#ifdef ACTUATOR_RUDDER2
  Sailboat::Instance()->getRudder2()->applyCommand(rudder/4.5);
#endif
}
