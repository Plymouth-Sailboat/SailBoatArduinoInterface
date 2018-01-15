/**
 * @file
 * @brief Mathematical and Essentials Tools Header file
 *
 * Tools
 * 	This file contains every mathematics functions that I had to re-designed
 *  These functions could be used everywhere in the project
 *
 */


#ifndef TOOLS_H
#define TOOLS_H


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

// Feedback:
#include "Messages.h"

// Mathematical library:
#include <math.h>


// ####################################################
// #                                                  #
// #                    FUNCTIONS                     #
// #                                                  #
// ####################################################
/**
 * Function map which works with doubles
 *
 * @param x: input value which need to be mapped (double)
 * @param in_min: minimum input value (double)
 * @param in_max: maximum input value (double)
 * @param out_min: minimum output value (double)
 * @param out_max: maximum output value (double)
 *
 * @return a double included in the interval [out_min ; out_max]
 */
double mapf(double x, double in_min, double in_max, double out_min, double out_max);


/** Executes the square function
 *
 * @param x: value (double)
 */
double square2(double x);


/**
 * Determinant dimension 2
 *
 * @param a: vector 1 (double[2])
 * @param b: vector 2 (double[2])
 */
double det2(double a[2], double b[2]);


/**
 * Norm 2 of a vector in dimension 2
 *
 * @param vect: vector (double[2])
 */
double norm2(double vect[2]);


/**
 * Extract the sign of a value
 *
 * @param val: value (double)
 */
int sign(double val);


/**
 * Setup function
 *  Initialize the reset function on pin 42. For more info, see "wiring.h" file
 *
 * !!!!!!!! Beware !!!!!!!!:
 * You need to plug this pin after having switch on the Arduino board. 
 * Then you need to unplug it before uploading a new program.
 */
void InitResetArduino();


/**
 * Reset Function
 *  Send the reset order
 *
 * !!!!!!!! Beware !!!!!!!!:
 * You need to plug this pin after having switch on the Arduino board. 
 * Then you need to unplug it before uploading a new program.
 */
void ResetArduino();


#endif