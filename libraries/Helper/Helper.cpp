#include <Helper.h>
#include <Arduino.h>

// ####################################################
// #                                                  #
// #                    FUNCTIONS                     #
// #                                                  #
// ####################################################
//  ====================================================
//  =                                                  =
//  =                       MAPF                       =
//  =                                                  =
//  ====================================================
double mapf(double x, double in_min, double in_max, double out_min, double out_max) {
	// Log(1, F("mapf()"), F(""));
	
	// Security:
	x = constrain(x, in_min, in_max);

	// Function:
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


//  ====================================================
//  =                                                  =
//  =                     SQUARE 2                     =
//  =                                                  =
//  ====================================================
double square2(double x) {
  // Log(1, F("square2()"), F(""));
  return(x*x);
}


//  ====================================================
//  =                                                  =
//  =                      DET 2                       =
//  =                                                  =
//  ====================================================
double det2(double a[2], double b[2]) {
  // 2-dimensional expression of the determinant:
  return (a[0]*b[1] - b[0]*a[1]);
}


//  ====================================================
//  =                                                  =
//  =                     NORM 2                       =
//  =                                                  =
//  ====================================================
double norm2(double vect[2]) {
  // Norm 2 in dimension 2
  return (sqrt(square2(vect[0]) + square2(vect[1])));
}


//  ====================================================
//  =                                                  =
//  =                      SIGN                        =
//  =                                                  =
//  ====================================================
int sign(double val) {
  int temp = 0;

  // This function returns 1 when a figure is negative and 0 if it is not:
  temp = signbitf(val);

  if (temp == 1) {
    return (-1);
  }
  else {
    return (1);
  }
}


//  ====================================================
//  =                                                  =
//  =                 INIT RESET ARDUINO               =
//  =                                                  =
//  ====================================================
void InitResetArduino() {
#ifndef RESET_PIN
  // Generation of a compiler error:
  #error "RESET_PIN" NOT DECLARED! see file "config.h"
#endif

  digitalWrite(RESET_PIN, HIGH);
  delayMicroseconds(10);
  pinMode(RESET_PIN, OUTPUT);
}


//  ====================================================
//  =                                                  =
//  =                   RESET ARDUINO                  =
//  =                                                  =
//  ====================================================
void ResetArduino() {
  // Log(1, F("ResetArduino()"), F(""));

  // Message(F("RESET"), F(""), F(""), 1);

  // To allow time for the message to be printed:
  delay(10);  // consider that printing 1 character takes around 1 ms (at 9600 baud)
  
  // Reset:
  digitalWrite(RESET_PIN, LOW);
}