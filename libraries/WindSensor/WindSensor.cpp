#include <WindSensor.h>

void WindSensor::init(){
}

void WindSensor::updateMeasures(){
	//Log(1, F("SetupWindSensor()"), F(""));  // Done in the Setup
	// Safety:
#ifndef WIND_SENSOR_PIN
  // Generation of a compiler error:
  #error "WIND_SENSOR_PIN" NOT DECLARED! see file "Wiring.h"
#endif

	value = analogRead(WIND_SENSOR_PIN);
	// If no value / no sensor:
	if ((value < 49) || (value > 1000)){
		// This test might not detect if the signal wire is not linked but if it is activated, 
		//   there is 100% chances that it's true !
		//Warning(F("WindAngle"), F("No Wind Sensor/Value read or sensor in bad state!"));
	}
	else {
		// Feedback initialization:
    //Message(F("##\t"), F("Wind Sensor seems to be OK"), F(""), 1);
	}
	
	//Log(0, F("Wind sensor corrected value :"), String(value));

	// returns the angle, with reference to the boat:
	angle = mapf(value, WIND_SENSOR_MIN, WIND_SENSOR_MAX, ANGLE_MIN, ANGLE_MAX);  // The angle is now in the [0;+360] interval

   // To set the angle in the [-180;+180] interval
   if (angle > 180) {
      angle -= ANGLE_MAX;
  }
  angle = -angle;
}