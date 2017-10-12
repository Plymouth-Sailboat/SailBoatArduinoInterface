/**
 * @file
 * @brief Remote Controller Header file
 *
 * RC Module
 *  - Finds the maximum and minimum values for each stick
 *  - Then saves these values inside of the EEPROM for 
 * 
 * Calibration:
 *  By plotting all output values on the serial port, I found that the biggest range (the one I choose) for 
 *  both stick are:
 *    - THROTTLE: when the switch (vertical one, on the left side of the power button), is push
 *               down until it stops making some noise.
 *    - SIDEWAYS: when the switch (under the left stick), is turn to the left 
 *               side at its maximum (push it until it stops making some noise).
 */


#ifndef RCMODULE_H
#define RCMODULE_H


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
// Initial values in the case of corrupted data emission at the beginning:
extern unsigned int throttle,
                    rudder,
                    *p_throttle,  // Creations of associates pointers
                    *p_rudder;

// Constants to adjust: TODO: interface to adjust values
const int THROTTLE_MIN = 912,  // Left stick (moving from bottom to top)
          THROTTLE_MAX = 1880,
          SIDEWAYS_MIN = 1160,  // Right stick (moving from left to right)
          SIDEWAYS_MAX = 2153,
          ACCURACY = 20;  // To find the threshold for reset


// ####################################################
// #                                                  #
// #                     STRUCT                       #
// #                                                  #
// ####################################################
// To store RC data:
typedef struct RemoteStruct {
  unsigned int throttle,
               rudder,
               status;
}RemoteStruct, *p_RemoteStruct;


// ####################################################
// #                                                  #
// #               FUNCTIONS PROTOTYPES               #
// #                                                  #
// ####################################################
/**
 * Setup for RC receiver
 */
void RcModuleSetup();

/**
 * Loop that reads remote RC commands and store them into variables.
 * If this function returns true, the boat is in manual mode, else the boat is in autonomous mode.
 *
 * @param *p_throttle: Command sent through the remote controller - for the boom (pointer unsigned int)
 * @param *p_rudder: Command sent through the remote controller - for the rudder (pointer unsigned int)
 * @param lastVal: last output of this function - history (bool)
 *
 * @return a boolean value indicating if the system is in automatic (true) or manual mode (false)
 *
 * Additional comments:
 *  Beware ! I use a Joysway J4C05 4 channel transmitter with a J5C01R 5-channel receiver:
 *  The left stick signal is carried by the 3rd channel, the right stick by 1st channel
 *  and the channel 5 indicates if the remote is "on" or "off"
 */
bool RcModuleLoop(unsigned int *p_throttle, unsigned int *p_rudder, bool lastVal);


#endif
