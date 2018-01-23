#include "ReturnHome.h"
#include <Sailboat.h>

ReturnHome::ReturnHome(){
}
void ReturnHome::init() {
}
void ReturnHome::Control(const geometry_msgs::Twist& cmd) {
  double latinit = Sailboat::Instance()->getGPS()->getLatInit();
  double longinit = Sailboat::Instance()->getGPS()->getLongInit();
}
