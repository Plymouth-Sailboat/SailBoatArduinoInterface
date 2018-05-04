#ifndef ACTUATOR_H
#define ACTUATOR_H

#include <config-Sailboat.h>
#include <Arduino.h>
#include <Helper.h>
#include <Log.h>

#include <ros.h>


class Actuator{
	public:
		Actuator(){}
		
		virtual void init();
		virtual void applyCommand(double command) = 0;
		
	private:
};


class ActuatorROS : public Actuator{
	public:
		ActuatorROS(const char* name, ros::Msg* msg) : pub(name, msg), nh(NULL){}
		void init(){}
		virtual void init(ros::NodeHandle* n){n->advertise(pub); nh = n;}
		virtual void communicateData() = 0;
		
	protected:
		ros::Publisher pub;
		ros::NodeHandle* nh;
};

#endif