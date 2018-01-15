/**
 * @file
 * @brief Sail Header file
 *
 * Sail:
 *  - Functions for the angle of the sail (limits of the movement, actioning,...)
 *  - The winch moves thanks to a Hitec HS-785HB Winch Servo (https://www.servocity.com/hs-785hb-servo)
 *  - I didn't found any existing code to control it, so I decided to create mine. It may be not well done.
 *  
 * Warning:
 *   !!! If you have any adjustments to make, please change only constants non-defined by a formula!
 */


#ifndef SAIL_H
#define SAIL_H


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
#include <math.h>
#include <Servo.h>

// Wiring map:
#include "Wiring.h"

// Feedback:
#include "Messages.h"

//Tools:
#include "Tools.h"


// ####################################################
// #                                                  #
// #                    VARIABLES                     #
// #                                                  #
// ####################################################
extern const unsigned int SAIL_MIN,
                          SAIL_NEUTRAL,
                          SAIL_MAX;


// ####################################################
// #                                                  #
// #                    FUNCTIONS                     #
// #                                                  #
// ####################################################
/**
 * @name Main functions
 * @{
 */
//  ====================================================
//  =                                                  =
//  =                   MAIN FUNCTIONS                 =
//  =                                                  =
//  ====================================================
// THESE FUNCTIONS ARE WHAT YOU'RE SUPPOSED TO USE.
//  CONSIDER THE FOLLOWING FUNCTIONS AS IF THEY WERE DECLARED AS PUBLIC IN AN 
//  OBJECT ORIENTED LANGUAGE.

/**
* Sets up the winch / sail configuration and mapping
*/
void SailSetup();


/**
 * Computes the desired angle of the sail
 *  The angle generated is relative to the command interval
 *
 * @param command: Command sent by the remote-controller
 * @param commandMin: Command minimum that can be sent to the sail
 * @param commandMax: Command maximum that can be sent to the sail
 */
double SailComputeCommand(int command, int commandMin, int commandMax);


/**
 * Moves the sail to a wanted position
 *
 * @param sailCommand: Desired angle of the sail (double)
 */
void SailApplyCommand(double sailCommand);

/**
 * @}
 */


/**
 * @name Sub functions
 * @{
 */
//  ====================================================
//  =                                                  =
//  =                   SUB FUNCTIONS                  =
//  =                                                  =
//  ====================================================
// THESE FUNCTIONS ARE USED BY MAIN FUNCTIONS, SO YOU'RE NOT SUPPOSED TO USE IT.
//  CONSIDER THE FOLLOWING FUNCTIONS AS IF THEY WERE DECLARED AS PRIVATE IN AN 
//  OBJECT ORIENTED LANGUAGE.
/**
 * Function that make winch orientation correspond to his command
 *
 * @param angle: unsigned int indicating the wanted angle, between 0 and 2826
 */
void Winch(unsigned int angle);


/*
 * Function that make winch orientation correspond to his command then give back control
 * approximately after the movement.
 * 
 * @param angle: unsigned int indicating the wanted angle, between 0 and 2826
 */
//void WinchSecure(unsigned int angle);

/**
 * @}
 */


#endif
