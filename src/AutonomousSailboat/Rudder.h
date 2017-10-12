/**
 * @file
 * @brief Rudder Header file
 *
 * Rudder:
 *  - Functions for the rudder (limits of the movement, actioning,...)
 *  - The rudder moves thanks to a Hitec HS 322 HD (https://www.servocity.com/hs-322hd-servo)
 */


#ifndef RUDDER_H
#define RUDDER_H


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
#include <Servo.h>

// Wiring map:
#include "Wiring.h"

// Feedback:
#include "Messages.h"


// ####################################################
// #                                                  #
// #                    VARIABLES                     #
// #                                                  #
// ####################################################
extern const unsigned int RUDDER_POS_NEUTRAL;

extern const double RUDDER_MIN,
                    RUDDER_NEUTRAL,
                    RUDDER_MAX;


// ####################################################
// #                                                  #
// #                    FUNCTIONS                     #
// #                                                  #
// ####################################################
/**
* Sets up the rudder configuration and mapping
*/
void RudderSetup();


/**
 * Computes the desired angle of the rudder 
 * (this angle consider that the rudder is in neutral position at 0° and can go from -Pi/4 to Pi/4)
 *  The command generated is relative to the command interval
 *
 * @param command: Command sent by the remote-controller
 * @param commandMin: Command minimum that can be sent through the remote
 * @param commandMax: Command maximum that can be sent through the remote
 */
double RudderComputeCommand(int command, int commandMin, int commandMax);


/**
 * Moves the rudder to a wanted position
 *
 * @param rudderCommand: Desired angle of the rudder (unsigned int)
 */
void RudderApplyCommand(double rudderCommand);


#endif
