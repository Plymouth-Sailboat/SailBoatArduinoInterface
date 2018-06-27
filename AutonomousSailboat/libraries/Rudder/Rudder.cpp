#include <Rudder.h>

void Rudder::init(ros::NodeHandle* n){
	//Log(1, F("RudderSetup()"), F(""));  // Done in the setup of AutonomousSailBoat.ino
	// Safety:
	#ifndef RUDDER_PIN
	// Generation of a compiler error:
	#error "RUDDER_PIN" NOT DECLARED! see file "Wiring.h"
#endif

	//rudder.attach(RUDDER_PIN, RUDDER_PWM_MIN, RUDDER_PWM_MAX);  // attaches the servo on pin 9 to the servo object
	rudder.attach(pin);
	
	// Set the rudder at the Neutral position
	rudder.write(posNeutral);
	
	ActuatorROS::init(n);
}

void Rudder::applyCommand(double command){
//	Logger::Log(1, F("RudderApplyCommand()"), F(""));

	unsigned int rudderCommandExact = posNeutral;
    if(command > anglemax)
        command = anglemax;
    if(command < anglemin)
        command = anglemin;

	// Generates the exact command:
	rudderCommandExact = mapf(command, anglemin, anglemax, posMax, posMin);
//	Logger::Log(0, F("RudderCommandExact"), String(rudderCommandExact));

	// Set the servo at the wanted position:
	rudder.write(rudderCommandExact);
}

void Rudder::communicateData(){
	msg.data = mapf(rudder.read(), posMax, posMin, anglemin, anglemax);
	pub.publish(&msg);
}
