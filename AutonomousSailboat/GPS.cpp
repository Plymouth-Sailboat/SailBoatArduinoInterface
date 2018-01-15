/**
 * @file
 * @brief GPS Source file
 *
 * GPS
 *  - Receive the GPS frames and uses it to computes his position
 */


// ####################################################
// #                                                  #
// #                      HEADERS                     #
// #                                                  #
// ####################################################
#include "GPS.h"


// ####################################################
// #                                                  #
// #                    VARIABLES                     #
// #                                                  #
// ####################################################
const int GPS_BAUD_RATE = 9600;  // Serial communication with the GPS

// GPS variables:
TinyGPSPlus gps;  // The TinyGPS++ object

// Variables:
double GPS_latInit = 50.374401f,
       *p_GPS_latInit = &GPS_latInit,  // associated Pointer
       GPS_longInit =  -4.140790f,
       *p_GPS_longInit = &GPS_longInit,  // associated Pointer
       GPS_PosX = 0.0f,  // Cartesian location of the boat
       *p_GPS_PosX = &GPS_PosX,  // associated Pointer
       GPS_PosY = 0.0f,  // Cartesian location of the boat
       *p_GPS_PosY = &GPS_PosY;  // associated Pointer


// ####################################################
// #                                                  #
// #                    FUNCTIONS                     #
// #                                                  #
// ####################################################
//  ====================================================
//  =                                                  =
//  =                     GPS SETUP                    =
//  =                                                  =
//  ====================================================
void GPSSetup(double *p_GPS_latInit, double *p_GPS_longInit) {
  //Log(1, F("GPSSetup()"), F(""));  // Done in the setup of AutonomousSailBoat.ino
  int GPSFix = 0;
  
  // Initialization:
#ifndef GPS_RX
  #error "GPS_RX" NOT DECLARED! see file "Wiring.h"
#endif
  pinMode(GPS_RX, OUTPUT);

#ifndef GPS_TX
  #error "GPS_TX" NOT DECLARED! see file "Wiring.h"
#endif
  pinMode(GPS_TX, INPUT);
  
  Serial1.begin(GPS_BAUD_RATE);
  
  // Waiting for GPS fix:
  while(!GPSFix) {  // Wait for signal
    if (Serial1.available() > 0) {
      Message(F("##\t\tGPS:\twaiting"), F(""), F(""), 1);
      if (gps.encode(Serial1.read()) && gps.location.isValid()) {
        // Initialization of the GPS location:
        *p_GPS_latInit = gps.location.lat();
        *p_GPS_longInit = gps.location.lng();

        // Feedback:
        Message(F("##\t\tGPS:\tFix"), F(""), F(""), 1);
        Message(F("##\t"), F("GPS Initial Latitude:"), String(*p_GPS_latInit), 1);
        Message(F("##\t"), F("GPS Initial Longitude:"), String(*p_GPS_longInit), 1);

        // End of the loop / the initialization
        GPSFix = 1;  // Indicates success of the initialization
      }
    }
    if (millis() > 5000 && gps.charsProcessed() < 10) {
      // Feedback:
      Message(F("##\t\tGPS:\tNo Signal!"), F(""), F(""), 1);
    }
  }
}


//  ====================================================
//  =                                                  =
//  =                      GPS LOOP                    =
//  =                                                  =
//  ====================================================
void GPSLoop(double *p_GPS_PosX, double *p_GPS_PosY, double GPS_latInit, double GPS_longInit) {
  Log(1, F("GPSLoop()"), F(""));

  double latActual,
         longActual;
  
  if (Serial1.available() > 0) {
    if (gps.encode(Serial1.read())) {
      if (gps.location.isValid()) {
        // Catching location:
        latActual = gps.location.lat();  // In degrees
        longActual = gps.location.lng();  // In degrees
        
        // Feedback:
        Message("\t", "GPS:", "Fix", 1);
        Log(0, F("GPS lat:"), String(latActual));
        Log(0, F("GPS longNext:"), String(longActual));
        
        // Changing the reference:
        *p_GPS_PosX = (double)(EARTH_RADIUS*(latActual - GPS_latInit)*(DEG_TO_RAD)*cos(GPS_longInit)); // x = EARTH_RADIUS*(a2-a1)*(pi/180)*cos(b1)
        *p_GPS_PosY = (double)(EARTH_RADIUS*(longActual - GPS_longInit)*(DEG_TO_RAD)); // y = EARTH_RADIUS*(b2-b1)*pi/180
        
        // Feedback:
        Log(2, F("GPS X:"), String(*p_GPS_PosX));
        Log(2, F("GPS Y:"), String(*p_GPS_PosY));
      }
    }
    else {
      Message(F("\t"), F("GPS:"), F("Lost!"), 1);
    }
  }
}
