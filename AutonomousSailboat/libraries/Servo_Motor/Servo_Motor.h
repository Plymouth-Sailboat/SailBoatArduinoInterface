#ifndef SERVO_MOTOR_H
#define SERVO_MOTOR_H

#include <ActuatorInterface.h>
#include <Servo.h>
#include <std_msgs/Float32.h>

class Servo_Motor : public ActuatorROS{
	public:
		Servo_Motor(unsigned int pin, unsigned int pwmNeutral, unsigned int pwmMin, unsigned int pwmMax, float anglemin, float anglemax,const char* name = "motor")
		: ActuatorROS(name, &msg), pin(pin), pwmNeutral(pwmNeutral), pwmMin(pwmMin), pwmMax(pwmMax), anglemin(anglemin), anglemax(anglemax){}
		
		void init(ros::NodeHandle* n);
		void applyCommand(double command);
		void communicateData();
	private:
		std_msgs::Float32 msg;
		Servo motor;
		unsigned int pin;
		unsigned int pwmNeutral;
		unsigned int pwmMin;
		unsigned int pwmMax;
		float anglemin;
		float anglemax;
};

#endif