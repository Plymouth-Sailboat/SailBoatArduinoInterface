/**
 * @file
 * @brief Ultrasonic Sensors Header file
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


#ifndef ULTRASONIC_H
#define ULTRASONIC_H


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


// ####################################################
// #                                                  #
// #                       CLASS                      #
// #                                                  #
// ####################################################
/**
 * Class Ultrasonic
 */
class Ultrasonic {
	private:		
		const float _CM_DIVISOR = 2.0*27.6233;  // centimetre divisor parameter
		
		int _TRIG_PIN,  // Trigger Pin
		    _ECHO_PIN;  // Echo Pin

	public:
		/**
		 * Builder function
	     *
	     * @param triggerPin: trigger pin
	     * @param echoPin: echo pin
		 */
		Ultrasonic(int triggerPin, int echoPin);

		/**
		 * Catching data
		 *
		 * @return Ultrasonic sensor send and receive total time
		 */
		long timing();

		/**
		 * Computing data
		 *
		 * @param microsec: Ultrasonic sensor send and receive total time
		 *
		 * @return calculate distance
		 */
		float CalcDistance(long microsec);
};

// TODO: Add Setup and Loop functions


#endif
