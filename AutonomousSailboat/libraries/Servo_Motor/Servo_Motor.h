#ifndef SERVO_MOTOR_H
#define SERVO_MOTOR_H

#include <ActuatorInterface.h>
#ifdef SERVO_SHIELD
	#include <Wire.h>
	#include <Adafruit_PWMServoDriver.h>
#else
	#include <Servo.h>
#endif

#include <std_msgs/Float32.h>

class Servo_Motor : public ActuatorROS{
	public:
		Servo_Motor(unsigned int pin, unsigned int pwmNeutral, unsigned int pwmMin, unsigned int pwmMax, float anglemin, float anglemax,const char* name = "motor")
		: ActuatorROS(name, &msg), pin(pin), pwmNeutral(pwmNeutral), pwmMin(pwmMin), pwmMax(pwmMax), anglemin(anglemin), anglemax(anglemax){}
		
		#ifdef SERVO_SHIELD
		void setMotor(Adafruit_PWMServoDriver* motor_ptr){motor = motor_ptr;}
		#endif
		
		void init(ros::NodeHandle* n);
		void applyCommand(double command);
		void communicateData();
	private:
	
		void motorSetup(){
			#ifdef SERVO_SHIELD
			motor->begin();
			#else
			motor.attach(pin);
			#endif		
		}
		
		void motorWrite(unsigned int pwm){
			#ifdef SERVO_SHIELD
			motor->setPWM(pin,0,pwm);
			#else
			motor.write(pwm);
			#endif		
		}
	
		std_msgs::Float32 msg;
		
		#ifdef SERVO_SHIELD
		Adafruit_PWMServoDriver* motor;
		#else
		Servo motor;
		#endif
		
		unsigned int pin;
		unsigned int pwmNeutral;
		unsigned int pwmMin;
		unsigned int pwmMax;
		float anglemin;
		float anglemax;
};

#endif