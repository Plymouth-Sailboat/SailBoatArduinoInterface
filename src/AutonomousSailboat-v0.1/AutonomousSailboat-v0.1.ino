#include <Sailboat.h>
#include <Log.h>

#include "Controller.h"

void setup() {
  Logger::MessagesSetup();
  
  Sailboat::Instance()->init();

  Sailboat::Instance()->controller = new Controller(3);
  Sailboat::Instance()->controller->init();
}

void loop() {
  Sailboat::Instance()->updateSensors();
  Sailboat::Instance()->controller->Control();
}
