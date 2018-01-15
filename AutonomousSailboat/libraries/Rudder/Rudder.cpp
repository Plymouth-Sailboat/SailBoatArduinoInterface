#include <Rudder.h>

void Rudder::init(){
	//Log(1, F("RudderSetup()"), F(""));  // Done in the setup of AutonomousSailBoat.ino
	// Safety:
	#ifndef RUDDER_PIN
	// Generation of a compiler error:
	#error "RUDDER_PIN" NOT DECLARED! see file "Wiring.h"
#endif

	rudder.attach(RUDDER_PIN, RUDDER_PWM_MIN, RUDDER_PWM_MAX);  // attaches the servo on pin 9 to the servo object

	// Set the rudder at the Neutral position
	rudder.write(RUDDER_POS_NEUTRAL);
}

void Rudder::applyCommand(double command){
	Logger::Log(1, F("RudderApplyCommand()"), F(""));

	unsigned int rudderCommandExact = RUDDER_POS_NEUTRAL;

	// Generates the exact command:
	rudderCommandExact = mapf(command, RUDDER_MIN, RUDDER_MAX, RUDDER_POS_MAX, RUDDER_POS_MIN);
	Logger::Log(0, F("RudderCommandExact"), String(rudderCommandExact));

	// Set the servo at the wanted position:
	rudder.write(rudderCommandExact);
}