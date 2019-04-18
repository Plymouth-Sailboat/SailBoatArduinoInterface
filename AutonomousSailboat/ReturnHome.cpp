#include "ReturnHome.h"
#include <Sailboat.h>

ReturnHome::ReturnHome():ControllerInterface(5000),q(1){
}
void ReturnHome::init() {
  GPS* gps = Sailboat::Instance()->getGPS();
  posStart[0] = gps->getLat();
  posStart[1] = gps->getLong();
}
void ReturnHome::Control(const geometry_msgs::Twist& cmd) {
  GPS* gps = Sailboat::Instance()->getGPS();

  XSens* xsens = Sailboat::Instance()->getIMU();
  WindSensor* wind = Sailboat::Instance()->getWindSensor();
  
  float yaw = xsens->getHeadingYaw();
  float trueWind = wind->getMeasure()+yaw;

  float psi = M_PI/4.0;
  float ksi = M_PI/3.0;
  float r = 40;

  double posInit[2] = {gps->getLatInit(), gps->getLongInit()};
  double posAct[2] = {gps->getLat(), gps->getLong()};

  
  const long earth_r = 6371000;

  //double longDif = (posInit[1] - posActual[1])*M_PI/180.0;
  //double latDif = (posInit[0] - posActual[0])*M_PI/180.0;

  double latAct = posAct[0]*M_PI/180.0;
  double lat1 = posStart[0]*M_PI/180.0;
  double lat2 = posInit[0]*M_PI/180.0;
  double longAct = posAct[1]*M_PI/180.0;
  double long1 = posStart[1]*M_PI/180.0;
  double long2 = posInit[1]*M_PI/180.0;

  //Get line distance
  double b[3] = {earth_r*cos(lat2)*cos(long2), earth_r*cos(lat2)*sin(long2)
                , earth_r*sin(lat2)};
  double a[3] = {earth_r*cos(lat1)*cos(long1), earth_r*cos(lat1)*sin(long1)
                , earth_r*sin(lat1)};
  double cur[3] = {earth_r*cos(latAct)*cos(longAct), earth_r*cos(latAct)*sin(longAct)
                , earth_r*sin(latAct)};
  double nb = sqrt(b[0]*b[0]+b[1]*b[1]+b[2]*b[2]);
  double na = sqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);

  double diflat = lat2-lat1;
  double diflong = long2-long1;
  double leng = (diflat*(latAct-lat1)+diflong*(longAct-long1))/(diflat*diflat+diflong*diflong);
  double currline[2] = {lat1 + diflat*leng, long1 + diflong*leng};

  float dksi = (latAct-currline[0]);
  float dlambda = (longAct-currline[1]);
  float ad = sin(dksi/2.0)*sin(dksi/2.0) + cos(currline[0])*cos(latAct)*sin(dlambda/2.0)*sin(dlambda/2.0);
  double distToLine = earth_r*atan2(sqrt(ad),sqrt(1-ad));
  
  double n[3] = {(a[1]*b[2]-a[2]*b[1])/(na*nb),(b[0]*a[2]-a[0]*b[2])/(na*nb)
                ,(a[0]*b[1]-a[1]*b[0])/(na*nb)};
  float e = n[0]*cur[0]+n[1]*cur[1]+n[2]*cur[2];
  e = e>0?distToLine:-distToLine;
  
  //Get bearing
  float yy = sin(long2-long1)*cos(lat2);
  float xx = cos(lat1)*sin(lat2)-sin(lat1)*cos(lat2)*cos(long2-long1);
  
  float phi = -atan2(yy,xx);
  float thetabar = phi - 2*psi/M_PI*atan(e/r);

  if(abs(e) > r/2.0)
    q = e>=0?1:-1;

  if(cos(trueWind-yaw)+cos(ksi) < 0 || (abs(e) < r && cos(trueWind-phi)+cos(ksi) <0))
    thetabar = M_PI + trueWind - q*ksi;

  double rudder = 0;
  double sail = 0;

  if (cos(yaw - thetabar) >= 0)
    rudder = RUDDER_MAX * sin(yaw - thetabar);
  else
    rudder = RUDDER_MAX * sign(sin(yaw - thetabar));

  sail = SAIL_MAX * (cos(trueWind-thetabar) + 1) / 2;

  Sailboat::Instance()->getRudder()->applyCommand(rudder);
  Sailboat::Instance()->getSail()->applyCommand(abs(sail));
#ifdef ACTUATOR_RUDDER2
  Sailboat::Instance()->getRudder2()->applyCommand(rudder/4.5);
#endif
}
