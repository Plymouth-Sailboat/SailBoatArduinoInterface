#include <Sailboat.h>
#include <Log.h>

#include "Controller.h"

ros::NodeHandle nh;

void setup() {
  
  Logger::MessagesSetup();
  
  nh.initNode();
  Sailboat::Instance()->init(nh);

  Sailboat::Instance()->setController(new Controller(3));
}

void loop() {
  Sailboat::Instance()->updateSensors();
  Sailboat::Instance()->communicateData();
  Sailboat::Instance()->Control();
}
