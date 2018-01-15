/**
 * @file
 * @brief GPS Header file
 *
 * GPS
 *  - Receive the GPS frames and uses it to computes his position
 */


#ifndef GPS_H
#define GPS_H


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

// External Headers:
#include <TinyGPS++.h>

// Wiring map:
#include "Wiring.h"

// Feedback:
#include "Messages.h"


// ####################################################
// #                                                  #
// #                    VARIABLES                     #
// #                                                  #
// ####################################################
// GPS:
extern double GPS_latInit,
              *p_GPS_latInit,
              GPS_longInit,
              *p_GPS_longInit,
              GPS_PosX,  // Cartesian location of the boat
              *p_GPS_PosX,
              GPS_PosY,  // Cartesian location of the boat
              *p_GPS_PosY;

const long EARTH_RADIUS = 6371000;  // Earth radius in metres


// ####################################################
// #                                                  #
// #                    FUNCTIONS                     #
// #                                                  #
// ####################################################
/**
 * Initialization of the GPS:
 *  - initialization of the chip
 *  - Wait for GPS fix
 *  - Set the initial position
 *
 * @param p_GPS_latInit pointer of the initial GPS latitude location
 * @param p_GPS_longInit pointer of the initial GPS longitude location
 */
void GPSSetup(double *p_GPS_latInit, double *p_GPS_longInit);


/**
 * Catches GPS signal and changing the reference of it
 *
 * @param p_GPS_PosX pointer to the GPS_PosX variable (double)
 * @param p_GPS_PosY pointer to the GPS_PosY variable (double)
 * @param p_GPS_latInit value of the initial GPS latitude location
 * @param p_GPS_longInit value of the initial GPS longitude location
 */
void GPSLoop(double *p_GPS_PosX, double *p_GPS_PosY, double p_GPS_latInit, double p_GPS_longInit);


#endif
