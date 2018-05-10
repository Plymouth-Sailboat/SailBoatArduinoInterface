#ifndef RUDDER_H
#define RUDDER_H

#include <ActuatorInterface.h>
#include <Servo.h>
#include <std_msgs/Float32.h>

class Rudder : public ActuatorROS{
	public:
		Rudder(const char* name = "rudder", unsigned int pin = RUDDER_PIN, unsigned int posNeutral = RUDDER_POS_NEUTRAL, unsigned int posMin = RUDDER_POS_MIN, unsigned int posMax = RUDDER_POS_MAX, float anglemin = RUDDER_MIN, float anglemax = RUDDER_MAX)
		: ActuatorROS(name, &msg), pin(pin), posNeutral(posNeutral), posMin(posMin), posMax(posMax), anglemin(anglemin), anglemax(anglemax){}
		
		void init(ros::NodeHandle* n);
		void applyCommand(double command);
		void communicateData();
	private:
		std_msgs::Float32 msg;
		Servo rudder;
		unsigned int pin;
		unsigned int posNeutral;
		unsigned int posMin;
		unsigned int posMax;
		float anglemin;
		float anglemax;
};

#endif