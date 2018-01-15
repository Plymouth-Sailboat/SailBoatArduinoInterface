#include <Sailboat.h>
#include <Log.h>

#include "Controller.h"

ros::NodeHandle nh;

void setup() {
  
  Logger::MessagesSetup();
  
  nh.initNode();
  Sailboat::Instance()->init(nh);

  ControllerInterface* controllers[1];
  controllers[0] = new Controller(3);
  
  Sailboat::Instance()->setControllers(controllers,1);
}

void loop() {
  Sailboat::Instance()->updateSensors();
  Sailboat::Instance()->communicateData();
  Sailboat::Instance()->Control();

  nh.spinOnce();
  delay(50);
}
