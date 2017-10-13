#ifndef SAIL_H
#define SAIL_H

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