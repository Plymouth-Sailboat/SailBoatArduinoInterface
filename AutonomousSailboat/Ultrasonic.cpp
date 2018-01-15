/**
 * @file
 * @brief Ultrasonic Sensors Source file
 *
 * Ultrasonic
 * 	Library for HC-SR04 Ultrasonic sensor module
 *
 * 	Adapted from : https://github.com/Make-The-Light-Sing/HCSR04UltraSonic
 *
 * TODO: If you decide to use ultrasonic sensors, you need to make a fusion with the 
 * 		 roll and pitch values of the IMU to be sure that the water is not detected.
 *       But it won't assure you that you don't see a wave :(
 */



// ####################################################
// #                                                  #
// #                      HEADERS                     #
// #                                                  #
// ####################################################
#include "Ultrasonic.h"


// ####################################################
// #                                                  #
// #                    FUNCTIONS                     #
// #                                                  #
// ####################################################
//  ====================================================
//  =                                                  =
//  =                    ULTRASONIC                    =
//  =                                                  =
//  ====================================================
Ultrasonic::Ultrasonic(int triggerPin,int echoPin) {
	_TRIG_PIN = triggerPin;
	_ECHO_PIN = echoPin;
	pinMode(_TRIG_PIN,OUTPUT);
	pinMode(_ECHO_PIN,INPUT);
}


//  ====================================================
//  =                                                  =
//  =                      TIMING                      =
//  =                                                  =
//  ====================================================
long Ultrasonic::timing() {
	digitalWrite(_TRIG_PIN,LOW);
	delayMicroseconds(2);
	digitalWrite(_TRIG_PIN,HIGH);
	delayMicroseconds(10);
	digitalWrite(_TRIG_PIN,LOW);
	return (pulseIn(_ECHO_PIN,HIGH));
}


//  ====================================================
//  =                                                  =
//  =                   CALC DISTANCE                  =
//  =                                                  =
//  ====================================================
float Ultrasonic::CalcDistance(long microsec) {
	float dist = microsec/_CM_DIVISOR;
	
	if (dist > 400) {  // Not reliable if dist > 400 cm, so I restrict it
		dist = 400;
	}
	return (dist);
}

// TODO: Add Setup and Loop functions