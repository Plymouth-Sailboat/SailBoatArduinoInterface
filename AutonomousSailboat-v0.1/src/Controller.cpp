/**
   @file
   @brief   Boat controller using a line-following method Source file

   Sail-boat controller for autonomous sail-boat - annotated version
   Algorithm: https://www.ensta-bretagne.fr/jaulin/paper_jaulin_irsc12.pdf

   This controller is realized with approximations like the fact that locally the earth is flat

   This approximation works as long as the boat's journey does not exceed 100km, if it does,
   you need to change the way the GPS coordinates are flatten.
   @sa GPS.h
   @sa GPS.cpp
*/

// ####################################################
// #                                                  #
// #                      HEADERS                     #
// #                                                  #
// ####################################################
#include "Controller.h"
#include <Sailboat.h>


// ####################################################
// #                                                  #
// #                    VARIABLES                     #
// #                                                  #
// ####################################################
#ifdef INTEGRATOR
// Integrator: TODO - tuning these values

float dt = 10.0f,  // Sampling time
      z = 0.0f;  // Value which will be changed every time the controller will be executed - must be set to 0
// when we change the aim direction

// gain of the integrator == amount of meter which needs a correction each dt:
const float ALPHA = 0.001f;
#endif

// width of the corridor in meters:
const int CORRIDOR = 50;

// Constants:
const double DELTA_RUDDER_MAX = RUDDER_MAX,  // PI/4 but in degrees
             GAMA = M_PI_4,  // PI/4  Incidence angle
             ZETA = 1.0471975511965976f;  // PI/3


// ####################################################
// #                                                  #
// #                    FUNCTIONS                     #
// #                                                  #
// ####################################################
//  ====================================================
//  =                                                  =
//  =                 CONTROLLER SETUP                 =
//  =                                                  =
//  ====================================================

Controller::Controller(int waypoints) : NB_POS_GPS(waypoints), token(1), finished(false) {
  initialPath = new double*[waypoints];
  newPath = new double*[waypoints + 1];
  for (int i = 0; i < 2; ++i) {
    newPath[i] = new double[2];
  }

  initialPath[0] = new double[2] {50.374395, -4.140784};
  initialPath[1] = new double[2] {50.375096, -4.140455};
  initialPath[2] = new double[2] {50.375195, -4.141362};
}


void Controller::init() {  // TODO
  //Log(1, F("ControllerSetup()"), F(""));  // Done in the setup of AutonomousSailBoat.ino
  // TODO:
  // File path = SD.open("PATH.TXT", FILE_READ);

  // if (path) {
  //   // read from the file until there's nothing else in it:
  //   while (path.available()) {
  //     //Serial.write(path.read());
  //   }
  //   // close the file:
  //   path.close();
  // } else {
  //   // if the file didn't open, print an error:
  //   Error(F("pathFollowing"), F("error opening path.txt"));
  // }

  // Pre-processing: Changing the reference

  Sailboat* boat = Sailboat::Instance();

  // First location:
  newPath[0][0] = 0.0f;  // GPS_latInit in the new reference
  newPath[0][1] = 0.0f;  // GPS_longInit in the new reference
  //Log(0, F("newPath:"), String(newPath[0][0]) + " " + String(newPath[0][1]));
  for (int i = 0 ; i < (NB_POS_GPS) ; i ++) {
    // Same algorithm than in GPS.cpp:
    newPath[i + 1][0] = (double)(EARTH_RADIUS * (initialPath[i][0] - boat->getGPS()->getLatInit()) * (DEG_TO_RAD) * cos(boat->getGPS()->getLongInit())); // x = EARTH_RADIUS*(a2-a1)*(pi/180)*cos(b1)
    newPath[i + 1][1] = (double)(EARTH_RADIUS * (initialPath[i][1] - boat->getGPS()->getLongInit()) * (DEG_TO_RAD)); // y = EARTH_RADIUS*(b2-b1)*pi/180
    //Log(0, F("newPath:"), String(newPath[i + 1][0]) + " " + String(newPath[i + 1][1]));
  }
}


//  ====================================================
//  =                                                  =
//  =                   CONTROLLER                     =
//  =                                                  =
//  ====================================================
void Controller::Control() {
  double posActual[2] = {Sailboat::Instance()->getGPS()->getX(), Sailboat::Instance()->getGPS()->getY()};
  pathFollowing();

  double posA[2] = {newPath[token - 1][0], newPath[token - 1][1]};
  double posB[2] = {newPath[token][0], newPath[token][1]};

  double windAngle = Sailboat::Instance()->getWindSensor()->getMeasure();
  double heading = Sailboat::Instance()->getIMU()->getHeading();
  double rudder = 0;
  double sail = 0;
  
  //Log(1, F("Controller()"), F(""));

  // tack indicator (its sign indicates the side of the tack):
  int q = 0;

  // Algebraic distance of the boat to the line:
  double vect1[2],
         vect2[2],
         temp[2],
         headingStar,
         headingBar,
         e,
         aim;

  if (!finished) {
    // vect1 = (posB - posA)/norm(posB - posA)
    temp[0] = posB[0] - posA[0];
    temp[1] = posB[1] - posA[1];

    vect1[0] = temp[0] / norm2(temp);
    vect1[1] = temp[1] / norm2(temp);

    // vect2 = posActual - posA
    vect2[0] = posActual[0] - posA[0];
    vect2[1] = posActual[1] - posB[0];

    e = det2(vect1, vect2);
    // The sign of 'e' indicates the side of the line, on which we are: e < 0 i.e. right side
    //                                                                  e > 0 i.e. left side
    //Log(0, F("e: "), String(e));

    if (abs(e) > CORRIDOR / 2) { // Need to tack:
      q = sign(e);  // choosing the side on which we will tack
    }
    //Log(0, F("q: "), String(q));

    // Angle of the aim (line between 'posA' and 'posB'), in the geographical reference,
    // so the angle is with respect to the East:
    aim = atan2((posB[0] - posA[0]), (posB[1] - posA[1]));
    //Log(0, F("aim: "), String(aim));

    // Correction of the trajectory, with the attraction of the line:
#ifdef INTEGRATOR
    // Integrator: TODO - adjustments
    z = z + ALPHA * dt * e;
    headingStar = aim - GAMA * atan((e + z) / CORRIDOR) / HALF_PI;
#else
    // No integrator:
    headingStar = aim - GAMA * atan(e / CORRIDOR) / HALF_PI;
#endif
    //Log(0, F("headingStar: "), String(headingStar));

    if (((cos(windAngle - headingStar) + cos(ZETA)) < 0)  //
        || ((abs(e) < CORRIDOR) && ((cos(windAngle - aim) + cos(ZETA)) < 0))) {  //
      // Corrected command:
      headingBar = PI + windAngle - q * ZETA;
    }
    else {
      headingBar = headingStar;  // The command does not need correction
    }
    //Log(0, F("headingBar: "), String(headingBar));

    // Command for the rudder:
    if (cos(heading - headingBar) >= 0) {
      // Soft command for the rudder:
      rudder = DELTA_RUDDER_MAX * sin(heading - headingBar);
    }
    else {
      // Bang bang command:
      rudder = DELTA_RUDDER_MAX * sign(sin(heading - headingBar));
    }

    sail = HALF_PI * RAD_TO_DEG * (cos(windAngle - headingBar) + 1) / 2;
  }
  else {
    // TODO: add   position keeping algorithm

    // Otherwise: We let the boat do what she wants
    rudder = RUDDER_MAX;
    sail = SAIL_MIN;

    //Message(F("####################################################\n3\n3"), F(""), F(""), 0);
    //Message(F("##       !!!       "), F("Finished"), F("       !!!       ##"), 1);
    //Message(F("####################################################\n3\n3"), F(""), F(""), 0);
  }
  Sailboat::Instance()->getRudder()->applyCommand(rudder);
  Sailboat::Instance()->getSail()->applyCommand(sail);
}


//  ====================================================
//  =                                                  =
//  =                  PATH FOLLOWING                  =
//  =                                                  =
//  ====================================================
void Controller::pathFollowing() {  // TODO
  //Log(1, F("pathFollowing()"), F(""));
  double posActual[2] = {Sailboat::Instance()->getGPS()->getX(), Sailboat::Instance()->getGPS()->getY()};

  // Finding the distance between the boat and the next way-point:
  double dist = square2(posActual[0] - newPath[token][0]) + square2(posActual[1] - newPath[token][1]);
  dist = sqrt(dist);

  while (dist <= CORRIDOR) {
    // Changing the aim:
    token = token + 1;  // We can't do "*p_token ++;" with pointers!

    // Checking the next Way-point:
    dist = square2(posActual[0] - newPath[token][0]) + square2(posActual[1] - newPath[token][1]);
    dist = sqrt(dist);
  }
  //Log(0, F("dist to next waypoint: "), String(dist));
  //Log(0, F("token: "), String(token));

  if (token > (NB_POS_GPS + 1)) {
    finished = true;
  }
}
