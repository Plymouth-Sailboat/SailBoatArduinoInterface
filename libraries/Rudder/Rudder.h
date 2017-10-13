#ifndef RUDDER_H
#define RUDDER_H

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