/**
 * @file
 * @brief Wind Sensor Header file
 *
 * WindSensor:
 *  - Function to initialize the Wind Sensor
 *	- Function made to read the wind-sensor's value
 */


#ifndef WIND_SENSOR_H
#define WIND_SENSOR_H


// ####################################################
// #                                                  #
// #                      DEFINE                      #
// #                                                  #
// ####################################################
// If the hardware tuning (the screw works, uncomment it), otherwise the software tuning is enabled:
//#define HARDWARE_TUNING


// ####################################################
// #                                                  #
// #                      HEADERS                     #
// #                                                  #
// ####################################################
// Arduino Headers: For using it with eclipse or processing
#if defined(ARDUINO) && ARDUINO >= 100
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif

// Wiring map:
#include "Wiring.h"

// Tools:
#include "Tools.h"

// Feedback:
#include "Messages.h"


// ####################################################
// #                                                  #
// #                    VARIABLES                     #
// #                                                  #
// ####################################################
// For the main program:
extern double WindOrientation;


// ####################################################
// #                                                  #
// #                     FUNCTIONS                    #
// #                                                  #
// ####################################################
/**
 * Function that tests if the wind sensor seems to be working
 *	This function is not perfect, but if the warning appears there is 100% chances that it's true !
 *
 */
void SetupWindSensor();

/**
 * Function that uses the wind sensor to find the angle of the wind in the boat's reference
 *
 */
double WindAngle();



#endif
