#include <Sailboat.h>
#include <Log.h>

#include "Controller.h"

void setup() {
  Logger::MessagesSetup();
  
  Sailboat::Instance()->init();

  Sailboat::Instance()->setController(new Controller(3));
}

void loop() {
  Sailboat::Instance()->updateSensors();
  Sailboat::Instance()->Control();
}
