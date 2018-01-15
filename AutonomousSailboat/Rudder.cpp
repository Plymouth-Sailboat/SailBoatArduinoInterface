/**
 * @file
 * @brief Rudder Source file
 *
 * Rudder:
 *  - Functions for the rudder (limits of the movement, actioning,...)
 *  - The rudder moves thanks to a Hitec HS 322 HD (https://www.servocity.com/hs-322hd-servo)
 */

// ####################################################
// #                                                  #
// #                      HEADERS                     #
// #                                                  #
// ####################################################
#include "Rudder.h"


// ####################################################
// #                                                  #
// #                    VARIABLES                     #
// #                                                  #
// ####################################################
Servo MyRudder;

// Rudder configuration positions:
const unsigned int RUDDER_POS_MIN = 45,//47,
                   RUDDER_POS_NEUTRAL = 90,//94,
                   RUDDER_POS_MAX = 135,//147,
                   RUDDER_PWM_MIN = 553,  // according to the seller
                   RUDDER_PWM_MAX = 2450;  // according to the seller

// these values are inverted on purpose:
const double RUDDER_MIN = -45,
             RUDDER_NEUTRAL = 0,
             RUDDER_MAX = 45;


// ####################################################
// #                                                  #
// #                    FUNCTIONS                     #
// #                                                  #
// ####################################################
//  ====================================================
//  =                                                  =
//  =                    RUDDER SETUP                  =
//  =                                                  =
//  ====================================================
void RudderSetup() {
  //Log(1, F("RudderSetup()"), F(""));  // Done in the setup of AutonomousSailBoat.ino
  // Safety:
  #ifndef RUDDER_PIN
    // Generation of a compiler error:
    #error "RUDDER_PIN" NOT DECLARED! see file "Wiring.h"
  #endif

  MyRudder.attach(RUDDER_PIN, RUDDER_PWM_MIN, RUDDER_PWM_MAX);  // attaches the servo on pin 9 to the servo object

  // Set the rudder at the Neutral position
  MyRudder.write(RUDDER_POS_NEUTRAL);
}


//  ====================================================
//  =                                                  =
//  =               RUDDER COMPUTE COMMAND             =
//  =                                                  =
//  ====================================================
double RudderComputeCommand(int command, int commandMin, int commandMax) {
  Log(1, F("RudderComputeCommand()"), F(""));
  double rudderCommand = RUDDER_NEUTRAL;

  // Insures that the servo command is staked by the physical limits of the servo:
  rudderCommand = mapf(command, commandMin, commandMax, RUDDER_MAX, RUDDER_MIN);
  return (rudderCommand);
}


//  ====================================================
//  =                                                  =
//  =                RUDDER APPLY COMMAND              =
//  =                                                  =
//  ====================================================
void RudderApplyCommand(double rudderCommand) {
  Log(1, F("RudderApplyCommand()"), F(""));

  unsigned int rudderCommandExact = RUDDER_POS_NEUTRAL;

  // Generates the exact command:
  rudderCommandExact = mapf(rudderCommand, RUDDER_MIN, RUDDER_MAX, RUDDER_POS_MAX, RUDDER_POS_MIN);
  Log(0, F("RudderCommandExact"), String(rudderCommandExact));

  // Set the servo at the wanted position:
  MyRudder.write(rudderCommandExact);
}
