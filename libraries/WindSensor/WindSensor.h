#ifndef WIND_SENSOR_H
#define WIND_SENSOR_H

#include <SensorsInterface.h>

class WindSensor : public Sensor{
	public:
		WindSensor(){}
		
		void init();
		void updateMeasures();
		
		double getMeasure(){return angle;}
		
	private:
		double angle;
};

#endif