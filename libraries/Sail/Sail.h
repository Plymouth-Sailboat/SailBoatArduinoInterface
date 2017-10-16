#ifndef SAIL_H
#define SAIL_H

#define D_MAST_MAINSAIL_SHEET  390 // between 170 and 710  // Place where the sheet is attached
#define D_MAST_RING 310
#define D_RING_ROPE 20
#define D_WINCH_BOOM 70
#define ROPE_MAX 554
#define ROPE_MIN 0
//#define                  //WINCH_NO_LOAD_SPEED_4_8V = 1600,  // To make an estimation of the delay required after command when power supply is 4.8V - For function WinchSecure()
//#define                  //WINCH_NO_LOAD_SPEED_6V = 1400,  // To make an estimation of the delay required after command when power supply is 6V - For function WinchSecure()
#define SERVO_PRECISION 100  // To compute with a best precision (x 100 before operations 
                                           //  with integer, then /100 to have a float / double)

// Limits of the angles between the sail / boom and the boat:
#define SAIL_MIN 0
#define SAIL_NEUTRAL SAIL_MIN
#define SAIL_MAX 90
#define WINCH_ANGLE_MIN 0 // Physical limit of the servomotor
#define WINCH_ANGLE_MAX 2826  // Physical limit of the servomotor
#define WINCH_PWM_MIN 600  // according to the seller
#define WINCH_PWM_MAX 2400  // according to the seller

#include <ActuatorInterface.h>
#include <Servo.h>

const double ROPE_RING_MAX = sqrt(square(D_RING_ROPE) + square(ROPE_MAX)),
             WINCH_DIAMETER = 1.456*25.4,  // The datasheet value is in inches - this one is in millimetres
             WINCH_OFFSET = (2*(ROPE_MAX - sqrt(abs((ROPE_RING_MAX - D_WINCH_BOOM)*(ROPE_RING_MAX - D_WINCH_BOOM) - D_RING_ROPE*D_RING_ROPE)))/WINCH_DIAMETER)*RAD_TO_DEG,  // Offset for the winch command
             // Limits of our configuration:
             WINCH_MIN_CONFIG = (2*(ROPE_MAX - sqrt(abs((ROPE_RING_MAX - D_WINCH_BOOM)*(ROPE_RING_MAX - sqrt((D_MAST_MAINSAIL_SHEET*cos(SAIL_MIN*DEG_TO_RAD) - D_MAST_RING)*(D_MAST_MAINSAIL_SHEET*cos(SAIL_MIN*DEG_TO_RAD) - D_MAST_RING) 
                                 + (D_MAST_MAINSAIL_SHEET*sin(SAIL_MIN*DEG_TO_RAD))*(D_MAST_MAINSAIL_SHEET*sin(SAIL_MIN*DEG_TO_RAD)) + D_WINCH_BOOM*D_WINCH_BOOM)) - D_RING_ROPE*D_RING_ROPE)))/WINCH_DIAMETER)*RAD_TO_DEG - WINCH_OFFSET,
             WINCH_MAX_CONFIG = (2*(ROPE_MAX - sqrt(abs((ROPE_RING_MAX - D_WINCH_BOOM)*(ROPE_RING_MAX - sqrt((D_MAST_MAINSAIL_SHEET*cos(SAIL_MAX*DEG_TO_RAD) - D_MAST_RING)*(D_MAST_MAINSAIL_SHEET*cos(SAIL_MAX*DEG_TO_RAD) - D_MAST_RING) 
                                 + (D_MAST_MAINSAIL_SHEET*sin(SAIL_MAX*DEG_TO_RAD))*(D_MAST_MAINSAIL_SHEET*sin(SAIL_MAX*DEG_TO_RAD)) + D_WINCH_BOOM*D_WINCH_BOOM)) - D_RING_ROPE*D_RING_ROPE)))/WINCH_DIAMETER)*RAD_TO_DEG - WINCH_OFFSET;

								 
class Sail : public Actuator{
	public:
		Sail(){}
		
		void init();
		void applyCommand(double command);
	private:
	void Winch(unsigned int angle);
	Servo winch;
};

#endif