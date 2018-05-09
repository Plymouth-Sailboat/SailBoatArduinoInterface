#ifndef SAIL_H
#define SAIL_H
//#define                  //WINCH_NO_LOAD_SPEED_4_8V = 1600,  // To make an estimation of the delay required after command when power supply is 4.8V - For function WinchSecure()
//#define                  //WINCH_NO_LOAD_SPEED_6V = 1400,  // To make an estimation of the delay required after command when power supply is 6V - For function WinchSecure()
#define SERVO_PRECISION 100  // To compute with a best precision (x 100 before operations 
                                           //  with integer, then /100 to have a float / double)

// Limits of the angles between the sail / boom and the boat:

#include <ActuatorInterface.h>
#include <Servo.h>
#include <std_msgs/Float32.h>

/*const double ROPE_RING_MAX = sqrt(square(D_RING_ROPE) + square(ROPE_MAX)),
             WINCH_OFFSET = (2*(ROPE_MAX - sqrt(abs((ROPE_RING_MAX - D_WINCH_BOOM)*(ROPE_RING_MAX - D_WINCH_BOOM) - D_RING_ROPE*D_RING_ROPE)))/WINCH_DIAMETER)*RAD_TO_DEG,  // Offset for the winch command
             // Limits of our configuration:
             WINCH_MIN_CONFIG = (2*(ROPE_MAX - sqrt(abs((ROPE_RING_MAX - D_WINCH_BOOM)*(ROPE_RING_MAX - sqrt((D_MAST_MAINSAIL_SHEET*cos(SAIL_MIN*DEG_TO_RAD) - D_MAST_RING)*(D_MAST_MAINSAIL_SHEET*cos(SAIL_MIN*DEG_TO_RAD) - D_MAST_RING) 
                                 + (D_MAST_MAINSAIL_SHEET*sin(SAIL_MIN*DEG_TO_RAD))*(D_MAST_MAINSAIL_SHEET*sin(SAIL_MIN*DEG_TO_RAD)) + D_WINCH_BOOM*D_WINCH_BOOM)) - D_RING_ROPE*D_RING_ROPE)))/WINCH_DIAMETER)*RAD_TO_DEG - WINCH_OFFSET,
             WINCH_MAX_CONFIG = (2*(ROPE_MAX - sqrt(abs((ROPE_RING_MAX - D_WINCH_BOOM)*(ROPE_RING_MAX - sqrt((D_MAST_MAINSAIL_SHEET*cos(SAIL_MAX*DEG_TO_RAD) - D_MAST_RING)*(D_MAST_MAINSAIL_SHEET*cos(SAIL_MAX*DEG_TO_RAD) - D_MAST_RING) 
                                 + (D_MAST_MAINSAIL_SHEET*sin(SAIL_MAX*DEG_TO_RAD))*(D_MAST_MAINSAIL_SHEET*sin(SAIL_MAX*DEG_TO_RAD)) + D_WINCH_BOOM*D_WINCH_BOOM)) - D_RING_ROPE*D_RING_ROPE)))/WINCH_DIAMETER)*RAD_TO_DEG - WINCH_OFFSET;*/

								 
class Sail : public ActuatorROS{
	public:
		Sail() : ActuatorROS("sail", &msg){}
		
		void init(ros::NodeHandle* n);
		void applyCommand(double command);
		void communicateData();
	private:
		std_msgs::Float32 msg;
		Servo winch;
		//void Winch(unsigned int angle);
};

#endif