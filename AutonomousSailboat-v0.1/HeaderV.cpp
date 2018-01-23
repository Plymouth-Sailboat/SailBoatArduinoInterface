#include "Standby.h"
#include <Sailboat.h>

Header::Header(){
}
void Header::init() {
}
void Header::Control(const geometry_msgs::Twist& cmd) {
  double x = cmd.linear.x;
  double y = cmd.linear.y;
  double z = cmd.linear.z;
}
