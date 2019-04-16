#ifndef SENSOR_H
#define SENSOR_H

#include <config-Sailboat.h>
#include <Arduino.h>
#include <Helper.h>
#include <Log.h>

#include <ros.h>


class Sensor{
	public:
		Sensor(unsigned long period = 100) : period(period), timer(0){}
		
		virtual void init(){}
		void update(){if(millis() - timer > period){ updateMeasures(); timer = millis();}}
		void updateT(){if(millis() - timer > period){ updateTest(); timer = millis();}}
		virtual void updateMeasures() = 0;
		virtual void updateTest() = 0;
		unsigned int getRaw(){return value;}
		
		unsigned int getPeriod(){return period;}
	protected:
		unsigned int value;
		
		unsigned long period;
		unsigned long timer;
};

class SensorROS : public Sensor{
	public:
		SensorROS(const char* name, ros::Msg* msg, unsigned long period = 100, unsigned long comperiod = 100) : Sensor(period), pub(name, msg), nh(NULL), comperiod(comperiod), comTimer(0){}
		
		void init(){}
		virtual void init(ros::NodeHandle* n){n->advertise(pub); nh = n;}
		void communicate(){if(millis() - comTimer > comperiod){communicateData(); comTimer = millis();}}
		virtual void communicateData() = 0;
	protected:
		ros::Publisher pub;
		ros::NodeHandle* nh;
	private:
		unsigned long comperiod;
		unsigned long comTimer;
};

#endif