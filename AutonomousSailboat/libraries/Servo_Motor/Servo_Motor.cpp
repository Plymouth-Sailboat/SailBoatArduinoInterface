#include <Servo_Motor.h>

void Servo_Motor::init(ros::NodeHandle* n){
	//motor.attach(pin, posMin, posMax);  // attaches the servo on pin 9 to the servo object
	motor.attach(pin);
	
	// Set the rudder at the Neutral position
	motor.write(pwmNeutral);
	
	ActuatorROS::init(n);
}

void Servo_Motor::applyCommand(double command){
	unsigned int commandExact = pwmNeutral;
    if(command > anglemax)
        command = anglemax;
    if(command < anglemin)
        command = anglemin;

	// Generates the exact command:
	commandExact = mapf(command, anglemin, anglemax, pwmMin, pwmMax);
//	Logger::Log(0, F("MotorCommandExact"), String(commandExact));

	// Set the servo at the wanted position:
	motor.write(commandExact);
}

void Servo_Motor::communicateData(){
	msg.data = mapf(motor.read(), pwmMin, pwmMax, anglemin, anglemax);
	pub.publish(&msg);
}
