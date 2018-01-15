#ifndef SENSOR_H
#define SENSOR_H

#include <config.h>
#include <Arduino.h>
#include <Helper.h>
#include <Log.h>

#include <ros.h>


class Sensor{
	public:
		Sensor(const char* name, ros::Msg* msg) : pub(name, msg){}
		
		virtual void init(ros::NodeHandle& n){n.advertise(pub);}
		virtual void updateMeasures() = 0;
		virtual void communicateData() = 0;
		unsigned int getRawValue(){return value;}
	protected:
		unsigned int value;
		ros::Publisher pub;
};

#endif