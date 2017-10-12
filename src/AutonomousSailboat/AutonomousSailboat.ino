/**
 * @file
 * @brief Main program
 * @author fcebron
 * @date 11/08/2017
 * @version 1.0
 *
 * AutonomousSailboat
 *  Internship project which aim is to set an model sailing boat RC-laser of 1 meter length into an autonomous sail-boat
 *  This project contains a manual and autonomous modes.
 *
 * This code is modular so, I added a lot of macro to deactivate some features (see DEBUG)

@mainpage Autonomous Sail-boat with Arduino


LICENCE
---------
MIT License

Copyright (c) 2017 fcebron

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

\page PageNameAlias Name of the page


 */





// ####################################################
// #                                                  #
// #                      DEBUG                       #
// #                                                  #
// ####################################################
// Comment the following definition to deactivate some features:
// IMU:
//#define IMU_ACTIVATED

// GPS:
//#define GPS_ACTIVATED

// RC module:
#define RCMODULE_ACTIVATED

// Rudder:
#define RUDDER_ACTIVATED

// Sail:
#define SAIL_ACTIVATED

// Ultrasonic:
//  Not advised to use it on the water
//  Moreover, the code is uncompleted
//#define ULTRASONIC_ACTIVATED

// Wind-sensor:
#define WINDSENSOR_ACTIVATED

// Demo - Mode:
#define DEMO
#ifdef DEMO
  // Disabling GPS:
  #ifdef GPS_ACTIVATED
    #undef GPS_ACTIVATED
  #endif
  
  // Enabling IMU:
  #ifndef IMU_ACTIVATED
    #define IMU_ACTIVATED
  #endif

  // Enabling Wind vane:
  #ifndef WINDSENSOR_ACTIVATED
    #define WINDSENSOR_ACTIVATED
  #endif
#endif


// ####################################################
// #                                                  #
// #                      HEADERS                     #
// #                                                  #
// ####################################################
// Feedback
#include "Messages.h"

// Tools
#include "Tools.h"

// Controller
#include "Controller.h"

// RC
#ifdef RCMODULE_ACTIVATED
  #include "RcModule.h"
#endif

// Rudder
#ifdef RUDDER_ACTIVATED
  #include "Rudder.h"
#endif

// Sail
#ifdef SAIL_ACTIVATED
  #include "Sail.h"
#endif

// Wind Sensor
#ifdef WINDSENSOR_ACTIVATED
  #include "WindSensor.h"
#endif

// Ultrasonic sensors
#ifdef ULTRASONIC_ACTIVATED
  #include "Ultrasonic.h"
#endif

// IMU
#ifdef IMU_ACTIVATED
  #include "IMU.h"
#endif

// GPS
//#ifdef GPS_ACTIVATED  // The GPS emulator requires variables defined in this file
#include "GPS.h"
//#endif


// ####################################################
// #                                                  #
// #                    VARIABLES                     #
// #                                                  #
// ####################################################
bool manualMode = false;


#ifdef SAIL_ACTIVATED
double sailCommand = SAIL_NEUTRAL,  // Variable containing the sail command
       *p_sailCommand = &sailCommand;  // Associated pointer
#endif


#ifdef RUDDER_ACTIVATED
double rudderCommand = RUDDER_NEUTRAL,  // Variable containing the rudder command
       *p_rudderCommand = &rudderCommand;  // Associated pointer
#endif


// For the Controller:
double posActual[2],
       pathOld[2],
       path[2];


#ifndef GPS_ACTIVATED
const int DT = 500,  // time to perform a loop in ms
          SPEED = 10;  // speed in meters per seconds

double distanceCrossed = SPEED*DT/1000;  // distance in meters
#endif

#ifdef DEMO
double windBoat = 0.0f;
double ruddLast = 0.0,
       *p_ruddLast = &ruddLast;
#endif


// ####################################################
// #                                                  #
// #                    FUNCTIONS                     #
// #                                                  #
// ####################################################
//  ====================================================
//  =                                                  =
//  =                       SETUP                      =
//  =                                                  =
//  ====================================================
/**
 * Set-up function
 *  The project starts with this function which contain the initialization part for each module
 */
void setup() {
  // Reset feature:
  InitResetArduino();

  MessagesSetup();  // First initialization
  Message(F("\n3\n3\t####################################################"), F(""), F(""), 1);
  Message(F("## Setup:"), F("Initialization"), F("Begins"), 1);
  Message(F("##"), F(""), F(""), 1);
  Message(F("##"), F("Waiting:........"), F(" 1/10   Messages"), 1);


  Message(F("##"), F("Waiting:........"), F(" 2/10  RC Module"), 1);
#ifdef RCMODULE_ACTIVATED
  RcModuleSetup();  // Second initialization


  Message(F("##"), F("Waiting:........"), F(" 3/10     Rudder"), 1);

  #ifdef RUDDER_ACTIVATED
    RudderSetup();  // Third initialization
  #else
    Message(F("##\t\tRudder Deactivated!"), F(""), F(""), 0);
    delay(TIME_TO_READ_MESSAGE);  // Time to read because it is important
  #endif


  Message(F("##"), F("Waiting:........"), F(" 4/10       Sail"), 1);

  #ifdef SAIL_ACTIVATED
    SailSetup();  // Fourth initialization
  #else
    Message(F("##\t\tSail Deactivated!"), F(""), F(""), 0);
    delay(TIME_TO_READ_MESSAGE);  // Time to read because it is important
  #endif
  
#else
  Message(F("##\t\tRcModule Deactivated!"), F(""), F(""), 0);
  delay(TIME_TO_READ_MESSAGE);  // Time to read because it is important


  Message(F("##"), F("Waiting:........"), F(" 3/10     Rudder"), 1);
  Message(F("##\t\tRudder Deactivated!"), F(""), F(""), 0);
  delay(TIME_TO_READ_MESSAGE);  // Time to read because it is important


  Message(F("##"), F("Waiting:........"), F(" 4/10       Sail"), 0);
  Message(F("##\t\tSail Deactivated!"), F(""), F(""), 1);
  delay(TIME_TO_READ_MESSAGE);  // Time to read because it is important
#endif


  Message(F("##"), F("Waiting:........"), F(" 5/10         SD"), 1);
#ifdef SD_ACTIVATED
  SDSetup();  // Fifth initialization
#else
  Message(F("##\t\tSD Deactivated!"), F(""), F(""), 0);
  delay(TIME_TO_READ_MESSAGE);  // Time to read because it is important
#endif


  Message(F("##"), F("Waiting:........"), F(" 6/10 WindSensor"), 1);
#ifdef WINDSENSOR_ACTIVATED
  SetupWindSensor();  // Sixth initialization
#else
  Message(F("##\t\tWind Sensor Deactivated!"), F(""), F(""), 0);
  delay(TIME_TO_READ_MESSAGE);  // Time to read because it is important
#endif


  Message(F("##"), F("Waiting:........"), F(" 7/10 Ultrasonic"), 1);
#ifdef ULTRASONIC_ACTIVATED
  // TODO:
  // Ultrasonic Initialization function:
#else
  Message(F("##\t\tUltrasonic Sensors Deactivated!"), F(""), F(""), 0);
  delay(TIME_TO_READ_MESSAGE);  // Time to read because it is important
#endif


  Message(F("##"), F("Waiting:........"), F(" 8/10        IMU"), 1);
#ifdef IMU_ACTIVATED
  IMUSetup(p_IMU);  // Eighth initialization
#else
  Message(F("##\t\tIMU Deactivated!"), F(""), F(""), 0);
  delay(TIME_TO_READ_MESSAGE);  // Time to read because it is important
#endif


  Message(F("##"), F("Waiting:........"), F(" 9/10        GPS"), 1);
#ifdef GPS_ACTIVATED
  GPSSetup(p_GPS_latInit, p_GPS_longInit);  // Ninth initialization
#else
  Message(F("##\t\tGPS Deactivated!"), F(""), F(""), 0);
  delay(TIME_TO_READ_MESSAGE);  // Time to read because it is important
  
  // We start at the beginning of the path:
  *p_GPS_latInit = initialPath[0][0];
  *p_GPS_longInit = initialPath[0][1];
#endif

  Message(F("##"), F("Waiting:........"), F("10/10 Controller"), 1);
  ControllerSetup(initialPath, newPath);  // Tenth initialization

  Message(F("##"), F(""), F(""), 1);
  Message(F("## Setup:"), F("Initialization"), F("Completed !"), 1);
  Message(F("####################################################\n3\n3"), F(""), F(""), 1);
  Message(F(""), F("Mode:"), F("Automatic"), 1);
}




//  ====================================================
//  =                                                  =
//  =                       LOOP                       =
//  =                                                  =
//  ====================================================
/**
 * Loop function
 *  Infinite Loop which contains the main code
 */
void loop() {
  Log(1, F("loop()"), F(""));  // To show the beginning of the loop!

#ifdef GPS_ACTIVATED
  // Finding GPS location:
  GPSLoop(p_GPS_PosX, p_GPS_PosY, GPS_latInit, GPS_longInit);  // using pointers to change the value of the 2 first variables
  //Message(F(""), "GPS X:" + String(GPS_PosX), "GPS Y:" + String(GPS_PosY), 1);
#else
  *p_GPS_PosX = GPS_PosX + distanceCrossed*cos(heading);
  *p_GPS_PosY = GPS_PosY + distanceCrossed*sin(heading);
#endif
  Log(2, F("GPSX"), String(GPS_PosX));
  Log(2, F("GPSY"), String(GPS_PosY));

#ifdef IMU_ACTIVATED
  // Finding orientation:
  IMULoop(p_IMU, p_heading);
#else
  *p_heading = -ruddLast/2;  // DEMO
#endif
  Log(2, F("Heading:"), String(heading));

#ifdef WINDSENSOR_ACTIVATED
  // Reading Wind Sensor:
  WindOrientation = -WindAngle();  // Angle with regards to the boat
  #ifdef DEMO
    //WindOrientation -= heading;  // stores the value in a temporary array
  #endif
  WindOrientation += heading;  // To find the wind angle with regards to the North  // TODO: add heading
  //Message(F(""), F("Wind:"), String(WindOrientation), 1);
#else
  double WindOrientation = 0;
#endif
  Log(2, F("Wind"), String(WindOrientation));

#ifdef RCMODULE_ACTIVATED
  // Gathering orders from the Remote Controller:
  manualMode = RcModuleLoop(p_throttle, p_rudder, manualMode);  // using pointers to change the value of the 2 variables

  if (manualMode) {
  #ifdef RUDDER_ACTIVATED
    rudderCommand = RudderComputeCommand(rudder, SIDEWAYS_MIN, SIDEWAYS_MAX);  // CHANGED
  #endif

  #ifdef SAIL_ACTIVATED
    sailCommand = SailComputeCommand(throttle, THROTTLE_MIN, THROTTLE_MAX);
  #endif
  }

  else {  // Automatic Mode
    posActual[0] = GPS_PosX;
    posActual[1] = GPS_PosY;

    pathFollowing(newPath, posActual, p_token);

    pathOld[0] = newPath[token - 1][0];
    pathOld[1] = newPath[token - 1][1];

    path[0] = newPath[token][0];
    path[1] = newPath[token][1];
    Controller(posActual, heading, WindOrientation, pathOld, path, p_rudderCommand, p_sailCommand);    // TODO:uncommment
  }
#else
  double rudderCommand = RUDDER_NEUTRAL,
         sailCommand = SAIL_NEUTRAL;
#endif
  Log(2, F("Rudd"), String(rudderCommand));
  Log(2, F("Sail"), String(sailCommand));


#ifdef RUDDER_ACTIVATED
  // Executing Rudder's command:
  RudderApplyCommand(rudderCommand);
#endif


#ifdef SAIL_ACTIVATED
  // Executing Sail's/Boom's command:
  SailApplyCommand(sailCommand);
#endif


#ifdef SD_ACTIVATED
  // Print data on the SD card:
  SaveSD();
#endif


#ifdef DEMO
  // Demo:
  String line1,
         line2;
  line1 = "H:" + String(heading) + " S:" + String(sailCommand);
  line2 = "W:" + String(WindOrientation)    + " R:" + String(rudderCommand);

  Message(F("## Screen:"), line1, line2, 1);  // For the LCD Screen
  *p_ruddLast = rudderCommand;
#endif
}
