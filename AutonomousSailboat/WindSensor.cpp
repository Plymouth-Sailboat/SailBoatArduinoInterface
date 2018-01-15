/**
 * @file
 * @brief Wind Sensor Source file
 *
 * WindSensor:
 *  - Function to initialize the Wind Sensor
 *	- Function made to read the wind-sensor's value
 */


// ####################################################
// #                                                  #
// #                      HEADERS                     #
// #                                                  #
// ####################################################
#include "WindSensor.h"


// ####################################################
// #                                                  #
// #                    VARIABLES                     #
// #                                                  #
// ####################################################
// Limits of the possible values of the sensor:
const int WIND_SENSOR_MIN = 51,  // Corresponds approximately at 5% of the maximum readable (1023) 
          WIND_SENSOR_MAX = 972;  // Corresponds approximately at 95% of the minimum readable (1023)

#ifndef HARDWARE_TUNING
const int WIND_SENSOR_OFFSET = 198;  // May need to be tuned!
                                     // value which gives the zero angle
           							             // Manual tuning does not works, so I chose software tuning
#endif

const double ANGLE_MAX = 360,
             ANGLE_MIN = 0;

// For the main program:
double WindOrientation = 0;


// ####################################################
// #                                                  #
// #                     FUNCTIONS                    #
// #                                                  #
// ####################################################
//  ====================================================
//  =                                                  =
//  =                   WIND ANGLE                     =
//  =                                                  =
//  ====================================================
void SetupWindSensor() {
	//Log(1, F("SetupWindSensor()"), F(""));  // Done in the Setup
	// Safety:
#ifndef WIND_SENSOR_PIN
  // Generation of a compiler error:
  #error "WIND_SENSOR_PIN" NOT DECLARED! see file "Wiring.h"
#endif

	unsigned int sensorValue = 0;

	// Reading sensor:
	sensorValue = analogRead(WIND_SENSOR_PIN);
	// If no value / no sensor:
	if ((sensorValue < 49) || (sensorValue > 1000)){
		// This test might not detect if the signal wire is not linked but if it is activated, 
		//   there is 100% chances that it's true !
		Warning(F("WindAngle"), F("No Wind Sensor/Value read or sensor in bad state!"));
	}
	else {
		// Feedback initialization:
    Message(F("##\t"), F("Wind Sensor seems to be OK"), F(""), 1);
	}
}


//  ====================================================
//  =                                                  =
//  =                   WIND ANGLE                     =
//  =                                                  =
//  ====================================================
double WindAngle() {
	Log(1, F("WindAngle()"), F(""));
	
	// Value to return:	
	double angle = 0;

	// Variable to store the sensor's value:
	unsigned long sensorValue = 0;

	sensorValue = analogRead(WIND_SENSOR_PIN);

	// If no value / no sensor:
	if (sensorValue == 0) {
		// This test might not detect if the signal wire is not linked but if it is activated, 
		//   there is 100% chances that it's true !
		Warning(F("WindAngle"), F("No Wind Sensor/Value read"));
	}

	Log(0, F("Wind sensor initial value :"), String(sensorValue));

#ifndef HARDWARE_TUNING
	// Adjusting the zero value (the manual tuning does not works, so it is a software tuning):
	sensorValue += (WIND_SENSOR_MAX - WIND_SENSOR_OFFSET);

	// Insures that we stay in the good interval:
	sensorValue -= WIND_SENSOR_MIN;
	sensorValue = sensorValue%(WIND_SENSOR_MAX - WIND_SENSOR_MIN);
	sensorValue += WIND_SENSOR_MIN;
#endif
  
	Log(0, F("Wind sensor corrected value :"), String(sensorValue));

	// returns the angle, with reference to the boat:
	angle = mapf(sensorValue, WIND_SENSOR_MIN, WIND_SENSOR_MAX, ANGLE_MIN, ANGLE_MAX);  // The angle is now in the [0;+360] interval

   // To set the angle in the [-180;+180] interval
   if (angle > 180) {
      angle -= ANGLE_MAX;
  }
	
	return (angle);
}