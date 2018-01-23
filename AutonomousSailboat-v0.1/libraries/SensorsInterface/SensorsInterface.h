#ifndef SENSOR_H
#define SENSOR_H

#include <config.h>
#include <Arduino.h>
#include <Helper.h>
#include <Log.h>

#include <ros.h>


class Sensor{
	public:
		Sensor(const char* name, ros::Msg* msg, unsigned int period = 100) : pub(name, msg), period(period){}
		
		virtual void init(ros::NodeHandle& n){n.advertise(pub);}
		void update(){if(millis() - timer > period){ updateMeasures(); timer = millis();}}
		void updateT(){if(millis() - timer > period){ updateTest(); timer = millis();}}
		virtual void updateMeasures() = 0;
		virtual void updateTest() = 0;
		virtual void communicateData() = 0;
		unsigned int getRawValue(){return value;}
	protected:
		unsigned int value;
		ros::Publisher pub;
		
		unsigned int period;
		unsigned long timer;
};

#endif