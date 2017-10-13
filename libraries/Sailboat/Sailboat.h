#ifndef SAILBOAT_H
#define SAILBOAT_H

#include <config.h>

#include <SensorsInterface.h>
#include <WindSensor.h>
#include <GPS.h>

class Sailboat{
	public:
		Sailboat(){}
		~Sailboat();
		
		void init();
		void updateSensors();
		
		WindSensor* getWindSensor(){return (WindSensor*)sensors[SENSOR_WINDSENSOR];}
		GPS* getGPS(){return (GPS*)sensors[SENSOR_GPS];}
		
	private:
		Sensor* sensors[NB_SENSORS];
};

#endif