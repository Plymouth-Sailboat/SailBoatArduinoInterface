#ifndef ACTUATOR_H
#define ACTUATOR_H

#include <config.h>
#include <Arduino.h>
#include <Helper.h>


class Actuator{
	public:
		Actuator(){}
		
		virtual void init();
		virtual void applyCommand(double command) = 0;
		
	private:
};

#endif