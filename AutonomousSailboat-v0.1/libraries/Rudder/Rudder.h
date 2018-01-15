#ifndef RUDDER_H
#define RUDDER_H

#define RUDDER_POS_MIN  45//47,
#define RUDDER_POS_NEUTRAL  90//94,
#define RUDDER_POS_MAX  135//147,
#define RUDDER_PWM_MIN  553  // according to the seller
#define RUDDER_PWM_MAX  2450  // according to the seller

// these values are inverted on purpose:
#define RUDDER_MIN  -45
#define RUDDER_NEUTRAL  0
#define RUDDER_MAX  45

#include <ActuatorInterface.h>
#include <Servo.h>

class Rudder : public Actuator{
	public:
		Rudder(){}
		
		void init();
		void applyCommand(double command);
	private:
		Servo rudder;
};

#endif