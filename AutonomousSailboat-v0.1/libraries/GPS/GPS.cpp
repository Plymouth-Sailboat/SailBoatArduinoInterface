#include <GPS.h>

void GPS::init(){
  //Log(1, F("GPSSetup()"), F(""));  // Done in the setup of AutonomousSailBoat.ino
  bool GPSFix = false;
  
  // Initialization:
#ifndef GPS_RX
  #error "GPS_RX" NOT DECLARED! see file "config.h"
#endif
  pinMode(GPS_RX, OUTPUT);

#ifndef GPS_TX
  #error "GPS_TX" NOT DECLARED! see file "config.h"
#endif
  pinMode(GPS_TX, INPUT);
  
  //Serial1.begin(GPS_BAUD_RATE);
  
  // Waiting for GPS fix:
  while(!GPSFix) {  // Wait for signal
    //if (Serial1.available() > 0) {
//      Logger::Message(F("##\t\tGPS:\twaiting"), F(""), F(""), 1);
//      if (gps.encode(Serial1.read()) && gps.location.isValid()) {
        // Initialization of the GPS location:
        GPS_latInit = gps.location.lat();
        GPS_longInit = gps.location.lng();

        // Feedback:
  //      Logger::Message(F("##\t\tGPS:\tFix"), F(""), F(""), 1);
  //      Logger::Message(F("##\t"), F("GPS Initial Latitude:"), String(*p_GPS_latInit), 1);
  //      Logger::Message(F("##\t"), F("GPS Initial Longitude:"), String(*p_GPS_longInit), 1);

        // End of the loop / the initialization
        GPSFix = true;  // Indicates success of the initialization
      //}
  //  }
    if (millis() > 5000 && gps.charsProcessed() < 10) {
      // Feedback:
    //  Logger::Message(F("##\t\tGPS:\tNo Signal!"), F(""), F(""), 1);
    }
  }
}

void GPS::updateMeasures(){
//  Logger::Log(1, F("GPSLoop()"), F(""));
  
  //if (Serial1.available() > 0) {
    //if (gps.encode(Serial1.read())) {
      if (gps.location.isValid()) {
        // Catching location:
        GPS_lat = gps.location.lat();  // In degrees
        GPS_long = gps.location.lng();  // In degrees
        
        // Feedback:
  //      Logger::Message("\t", "GPS:", "Fix", 1);
  //      Logger::Log(0, F("GPS lat:"), String(latActual));
  //      Logger::Log(0, F("GPS longNext:"), String(longActual));
        
        // Changing the reference:
        GPS_PosX = (double)(EARTH_RADIUS*(GPS_lat - GPS_latInit)*(DEG_TO_RAD)*cos(GPS_longInit)); // x = EARTH_RADIUS*(a2-a1)*(pi/180)*cos(b1)
        GPS_PosY = (double)(EARTH_RADIUS*(GPS_long - GPS_longInit)*(DEG_TO_RAD)); // y = EARTH_RADIUS*(b2-b1)*pi/180
        
        // Feedback:
    //    Logger::Log(2, F("GPS X:"), String(*p_GPS_PosX));
    //    Logger::Log(2, F("GPS Y:"), String(*p_GPS_PosY));
      }
    //}
    //else {
    //  Logger::Message(F("\t"), F("GPS:"), F("Lost!"), 1);
    //}
  //}
}
