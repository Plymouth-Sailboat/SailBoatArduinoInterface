#ifndef RUDDER_H
#define RUDDER_H

#include <ActuatorInterface.h>
#include <Servo.h>
#include <std_msgs/Float32.h>

class Rudder : public ActuatorROS{
	public:
		Rudder() : ActuatorROS("rudder", &msg){}
		
		void init(ros::NodeHandle* n);
		void applyCommand(double command);
		void communicateData();
	private:
		std_msgs::Float32 msg;
		Servo rudder;
};

#endif