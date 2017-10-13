#include <Sailboat.h>

Sailboat boat;

void setup() {
  boat.init();
}

void loop() {
  boat.updateSensors();
}
