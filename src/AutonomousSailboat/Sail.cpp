/**
 * @file
 * @brief Sail Source file
 *
 * Sail:
 *  - Functions for the angle of the sail (limits of the movement, actioning,...)
 *  - The winch moves thanks to a Hitec HS-785HB Winch Servo (https://www.servocity.com/hs-785hb-servo)
 *  - I didn't found any existing code to control it, so I decided to create mine. It may be not well done.
 *  
 * Warning:
 *   !!! If you have any adjustments to make, please change only constants non-defined by a formula!
 */


// ####################################################
// #                                                  #
// #                      HEADERS                     #
// #                                                  #
// ####################################################
#include "Sail.h"


// ####################################################
// #                                                  #
// #                    VARIABLES                     #
// #                                                  #
// ####################################################
Servo MyWinch;

// Geometrical configuration of the mainsail: (all units are in millimetres)
const unsigned int D_MAST_MAINSAIL_SHEET = 390, // between 170 and 710  // Place where the sheet is attached
                   D_MAST_RING = 310,
                   D_RING_ROPE = 20,
                   D_WINCH_BOOM = 70,
                   ROPE_MAX = 554,
                   ROPE_MIN = 0,
                   //WINCH_NO_LOAD_SPEED_4_8V = 1600,  // To make an estimation of the delay required after command when power supply is 4.8V - For function WinchSecure()
                   //WINCH_NO_LOAD_SPEED_6V = 1400,  // To make an estimation of the delay required after command when power supply is 6V - For function WinchSecure()
                   SERVO_PRECISION = 100;  // To compute with a best precision (x 100 before operations 
                                           //  with integer, then /100 to have a float / double)

// Limits of the angles between the sail / boom and the boat:
const unsigned int SAIL_MIN = 0,
                   SAIL_NEUTRAL = SAIL_MIN,
                   SAIL_MAX = 90,
                   WINCH_ANGLE_MIN = 0,  // Physical limit of the servomotor
                   WINCH_ANGLE_MAX = 2826,  // Physical limit of the servomotor
                   WINCH_PWM_MIN = 600,  // according to the seller
                   WINCH_PWM_MAX = 2400;  // according to the seller

// NEVER TOUCH THESE CONSTANTS:
const double ROPE_RING_MAX = sqrt(square(D_RING_ROPE) + square(ROPE_MAX)),
             WINCH_DIAMETER = 1.456*25.4,  // The datasheet value is in inches - this one is in millimetres
             WINCH_OFFSET = (2*(ROPE_MAX - sqrt(abs((ROPE_RING_MAX - D_WINCH_BOOM)*(ROPE_RING_MAX - D_WINCH_BOOM) - D_RING_ROPE*D_RING_ROPE)))/WINCH_DIAMETER)*RAD_TO_DEG,  // Offset for the winch command
             // Limits of our configuration:
             WINCH_MIN_CONFIG = (2*(ROPE_MAX - sqrt(abs((ROPE_RING_MAX - D_WINCH_BOOM)*(ROPE_RING_MAX - sqrt((D_MAST_MAINSAIL_SHEET*cos(SAIL_MIN*DEG_TO_RAD) - D_MAST_RING)*(D_MAST_MAINSAIL_SHEET*cos(SAIL_MIN*DEG_TO_RAD) - D_MAST_RING) 
                                 + (D_MAST_MAINSAIL_SHEET*sin(SAIL_MIN*DEG_TO_RAD))*(D_MAST_MAINSAIL_SHEET*sin(SAIL_MIN*DEG_TO_RAD)) + D_WINCH_BOOM*D_WINCH_BOOM)) - D_RING_ROPE*D_RING_ROPE)))/WINCH_DIAMETER)*RAD_TO_DEG - WINCH_OFFSET,
             WINCH_MAX_CONFIG = (2*(ROPE_MAX - sqrt(abs((ROPE_RING_MAX - D_WINCH_BOOM)*(ROPE_RING_MAX - sqrt((D_MAST_MAINSAIL_SHEET*cos(SAIL_MAX*DEG_TO_RAD) - D_MAST_RING)*(D_MAST_MAINSAIL_SHEET*cos(SAIL_MAX*DEG_TO_RAD) - D_MAST_RING) 
                                 + (D_MAST_MAINSAIL_SHEET*sin(SAIL_MAX*DEG_TO_RAD))*(D_MAST_MAINSAIL_SHEET*sin(SAIL_MAX*DEG_TO_RAD)) + D_WINCH_BOOM*D_WINCH_BOOM)) - D_RING_ROPE*D_RING_ROPE)))/WINCH_DIAMETER)*RAD_TO_DEG - WINCH_OFFSET;


// ####################################################
// #                                                  #
// #                    FUNCTIONS                     #
// #                                                  #
// ####################################################
//  ====================================================
//  =                                                  =
//  =                     SAIL SETUP                   =
//  =                                                  =
//  ====================================================
void SailSetup() {
  //Log(1, F("SailSetup()"), F(""));  // Done in the setup of AutonomousSailBoat.ino
  // Safety:
  #ifndef WINCH_PIN
    // Generation of a compiler error:
    #error "WINCH_PIN" NOT DECLARED! see file "Wiring.h"
  #endif

  MyWinch.attach(WINCH_PIN, WINCH_PWM_MIN, WINCH_PWM_MAX);
  Winch(WINCH_MIN_CONFIG);
}


//  ====================================================
//  =                                                  =
//  =              SAIL COMPUTE COMMAND                =
//  =                                                  =
//  ====================================================
double SailComputeCommand(int command, int commandMin, int commandMax) {
  double sailCommand;  // Angle of the winch
  
  Log(1, F("SailComputeCommand()"), F(""));

  Log(0, F("Command:"), String(command));
  
  sailCommand = mapf(command, commandMin, commandMax, SAIL_MIN, SAIL_MAX);
  Log(0, F("SailCommand Map:"), String(sailCommand));

  if (sailCommand < 0) {
    Error(F("SailCommand()"), F("sailCommand is negative"));
    sailCommand = SAIL_MIN;
  }
  return(sailCommand);
}


//  ====================================================
//  =                                                  =
//  =                SAIL APPLY COMMAND                =
//  =                                                  =
//  ====================================================
void SailApplyCommand(double sailCommand) {
  double ropeCommand,  // length of the linear rope on top of the boat (just linked to the winch)
         sheetCommand,  // length of the sheet which is located between the boom and the ring on top of the boat
         winchCommand;  // Angle of the winch  // Changing degrees to radians for next computations:
  double temp1 = 0,
         temp2 = 0,
         temp3 = 0,
         temp4 = 0;
  
  Log(1, F("SailApplyCommand()"), F(""));

  /* Old way to do it - could be interesting
  // Vectorial formula to find the desired length of the rope:
  sheetCommand = square2(D_MAST_MAINSAIL_SHEET*cos(sailCommand) - D_MAST_RING); // x²
  Log(0, F("SheetCommand x2:"), String(sheetCommand));
  sheetCommand += square2(D_MAST_MAINSAIL_SHEET*sin(sailCommand));  // +y²
  Log(0, F("SheetCommand +y2:"), String(sheetCommand));
  sheetCommand += square2(D_WINCH_BOOM);  // +z²
  Log(0, F("SheetCommand +z2:"), String(sheetCommand));
  sheetCommand = sqrt(sheetCommand);  // sqrt(x² + y² + z²): length of the rope
  Log(0, F("SheetCommand:"), String(sheetCommand));*/


  sailCommand = sailCommand*DEG_TO_RAD;  //cosinus has to be working with radians
  Log(0, F("sailCommandRad:"), String(sailCommand));

  temp1 = square2(D_MAST_MAINSAIL_SHEET);
  Log(0, F("SheetCommand1:"), String(temp1));
  temp2 = square2(D_MAST_RING) + square2(D_WINCH_BOOM);
  Log(0, F("SheetCommand2:"), String(temp2));
  temp3 = 2*cos(sailCommand)*D_MAST_MAINSAIL_SHEET*D_MAST_RING;
  Log(0, F("SheetCommand3:"), String(temp3));
  temp3 = constrain(temp3, 0, 241800);  // Security - raw values because the computation is costly, but it is the 
                                        //            expression above with sailCommand at its min and max
  Log(0, F("SheetCommand3corr:"), String(temp3));
  temp4 = temp1 + temp2 - temp3;
  Log(0, F("SheetCommand4:"), String(temp4));
  sheetCommand = sqrt(temp4);  // length of the rope
  Log(0, F("SheetCommand:"), String(sheetCommand));


  // TODO: find why the 90° is not reached!

  // Pythagore's formula to find the desired angle of the winch to reach the desired length of the rope:
  ropeCommand = sqrt(abs(square2(ROPE_RING_MAX - sheetCommand) - square2(D_RING_ROPE)));
  ropeCommand = ROPE_MAX - ropeCommand;
  Log(0, F("RopeCommand:"), String(ropeCommand));

  // Safety limits: rope
  ropeCommand = constrain(ropeCommand, ROPE_MIN, ROPE_MAX);

  Log(0, F("RopeCommandSafe:"), String(ropeCommand));


  // Formula to find the link between the winch angle and the length of rope wound or unwound:
  winchCommand = 2*ropeCommand/WINCH_DIAMETER;
  winchCommand = winchCommand*RAD_TO_DEG - WINCH_OFFSET;
  Log(0, F("WinchCommand:"), String(winchCommand));

  // Safety limits: winch
  winchCommand = constrain(winchCommand, max(WINCH_MIN_CONFIG, WINCH_ANGLE_MIN), min(WINCH_MAX_CONFIG, WINCH_ANGLE_MAX));

  Log(0, F("WinchCommandSafe:"), String(winchCommand));
  
  // Set the servo at the wanted position:
  Winch((unsigned int) winchCommand);
}


//  ====================================================
//  =                                                  =
//  =                       WINCH                      =
//  =                                                  =
//  ====================================================
void Winch(unsigned int angle) {
  //Log(1, F("Winch()"), F(""));  // To have a clean Boot menu

  int pwmLengh;
  
  pwmLengh = map(angle, WINCH_ANGLE_MIN, WINCH_ANGLE_MAX, WINCH_PWM_MIN, WINCH_PWM_MAX); 
  MyWinch.writeMicroseconds(pwmLengh);
}

/*
//  ====================================================
//  =                                                  =
//  =                   WINCH SECURE                   =
//  =                                                  =
//  ====================================================
void WinchSecure(unsigned int angle) {
  Log(1, F("WinchSecure()"), F(""));

  int multiple,
      remainder;

  float rotationDuration = 0;

  // Duration estimation:
  multiple = angle/360;
  remainder = angle%360;
  rotationDuration = WINCH_NO_LOAD_SPEED_6V*(multiple + remainder/360);  // in milliseconds
  
  Winch(angle);
  delay(rotationDuration);  // Waiting for the end of the movement - roughly
}*/