#ifndef SENSOR_H
#define SENSOR_H

#include <config.h>
#include <Arduino.h>
#include <Helper.h>
#include <Log.h>

#include <ros.h>


class Sensor{
	public:
		Sensor(unsigned int period = 100) : period(period), timer(0){}
		
		virtual void init(){}
		void update(){if(millis() - timer > period){ updateMeasures(); timer = millis();}}
		void updateT(){if(millis() - timer > period){ updateTest(); timer = millis();}}
		virtual void updateMeasures() = 0;
		virtual void updateTest() = 0;
		unsigned int getRaw(){return value;}
	protected:
		unsigned int value;
		
		unsigned int period;
		unsigned long timer;
};

class SensorROS : public Sensor{
	public:
		SensorROS(const char* name, ros::Msg* msg, unsigned int period = 100) : Sensor(period), pub(name, msg), nh(NULL){}
		
		void init(){}
		virtual void init(ros::NodeHandle* n){n->advertise(pub); nh = n;}
		virtual void communicateData() = 0;
	protected:
		ros::Publisher pub;
		ros::NodeHandle* nh;
};

#endif