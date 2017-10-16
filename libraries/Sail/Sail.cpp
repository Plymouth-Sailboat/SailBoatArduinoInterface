#include <Sail.h>

void Sail::init(){
	//Log(1, F("SailSetup()"), F(""));  // Done in the setup of AutonomousSailBoat.ino
	// Safety:
#ifndef WINCH_PIN
	// Generation of a compiler error:
	#error "WINCH_PIN" NOT DECLARED! see file "Wiring.h"
#endif

	winch.attach(WINCH_PIN, WINCH_PWM_MIN, WINCH_PWM_MAX);
	Winch(WINCH_MIN_CONFIG);
}

void Sail::applyCommand(double command){
	
	double ropeCommand,  // length of the linear rope on top of the boat (just linked to the winch)
	sheetCommand,  // length of the sheet which is located between the boom and the ring on top of the boat
	winchCommand;  // Angle of the winch  // Changing degrees to radians for next computations:
	double temp1 = 0,
	temp2 = 0,
	temp3 = 0,
	temp4 = 0;

	Logger::Log(1, F("SailApplyCommand()"), F(""));

	/* Old way to do it - could be interesting
// Vectorial formula to find the desired length of the rope:
sheetCommand = square2(D_MAST_MAINSAIL_SHEET*cos(sailCommand) - D_MAST_RING); // x²
Log(0, F("SheetCommand x2:"), String(sheetCommand));
sheetCommand += square2(D_MAST_MAINSAIL_SHEET*sin(sailCommand));  // +y²
Log(0, F("SheetCommand +y2:"), String(sheetCommand));
sheetCommand += square2(D_WINCH_BOOM);  // +z²
Log(0, F("SheetCommand +z2:"), String(sheetCommand));
sheetCommand = sqrt(sheetCommand);  // sqrt(x² + y² + z²): length of the rope
Log(0, F("SheetCommand:"), String(sheetCommand));*/


	command = command*DEG_TO_RAD;  //cosinus has to be working with radians
	Logger::Log(0, F("sailCommandRad:"), String(sailCommand));

	temp1 = square2(D_MAST_MAINSAIL_SHEET);
	Logger::Log(0, F("SheetCommand1:"), String(temp1));
	temp2 = square2(D_MAST_RING) + square2(D_WINCH_BOOM);
	Logger::Log(0, F("SheetCommand2:"), String(temp2));
	temp3 = 2*cos(command)*D_MAST_MAINSAIL_SHEET*D_MAST_RING;
	Logger::Log(0, F("SheetCommand3:"), String(temp3));
	temp3 = constrain(temp3, 0, 241800);  // Security - raw values because the computation is costly, but it is the 
	//            expression above with sailCommand at its min and max
	Logger::Log(0, F("SheetCommand3corr:"), String(temp3));
	temp4 = temp1 + temp2 - temp3;
	Logger::Log(0, F("SheetCommand4:"), String(temp4));
	sheetCommand = sqrt(temp4);  // length of the rope
	Logger::Log(0, F("SheetCommand:"), String(sheetCommand));


	// TODO: find why the 90° is not reached!

	// Pythagore's formula to find the desired angle of the winch to reach the desired length of the rope:
	ropeCommand = sqrt(abs(square2(ROPE_RING_MAX - sheetCommand) - square2(D_RING_ROPE)));
	ropeCommand = ROPE_MAX - ropeCommand;
	Logger::Log(0, F("RopeCommand:"), String(ropeCommand));

	// Safety limits: rope
	ropeCommand = constrain(ropeCommand, ROPE_MIN, ROPE_MAX);

	Logger::Log(0, F("RopeCommandSafe:"), String(ropeCommand));


	// Formula to find the link between the winch angle and the length of rope wound or unwound:
	winchCommand = 2*ropeCommand/WINCH_DIAMETER;
	winchCommand = winchCommand*RAD_TO_DEG - WINCH_OFFSET;
	Logger::Log(0, F("WinchCommand:"), String(winchCommand));

	// Safety limits: winch
	winchCommand = constrain(winchCommand, max(WINCH_MIN_CONFIG, WINCH_ANGLE_MIN), min(WINCH_MAX_CONFIG, WINCH_ANGLE_MAX));

	Logger::Log(0, F("WinchCommandSafe:"), String(winchCommand));

	// Set the servo at the wanted position:
	Winch((unsigned int) winchCommand);
}

void Sail::Winch(unsigned int angle) {
  Logger::Log(1, F("Winch()"), F(""));  // To have a clean Boot menu

  int pwmLengh;
  
  pwmLengh = map(angle, WINCH_ANGLE_MIN, WINCH_ANGLE_MAX, WINCH_PWM_MIN, WINCH_PWM_MAX); 
  winch.writeMicroseconds(pwmLengh);
}