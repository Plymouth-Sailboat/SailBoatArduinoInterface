#ifndef SAILBOAT_H
#define SAILBOAT_H

#include <config.h>

#include <WindSensor.h>
#include <GPS.h>
#include <IMU.h>

#include <Rudder.h>
#include <Sail.h>

#include <ControllerInterface.h>

class Sailboat{
	public:
		Sailboat(){}
		~Sailboat();
		
		void init();
		void updateSensors();
		
		WindSensor* getWindSensor(){return (WindSensor*)sensors[SENSOR_WINDSENSOR];}
		GPS* getGPS(){return (GPS*)sensors[SENSOR_GPS];}
		IMU* getIMU(){return (IMU*)sensors[SENSOR_IMU];}
		
		Rudder* getRudder(){return (Rudder*)actuators[ACTUATOR_RUDDER];}
		Sail* getSail(){return (Sail*)actuators[ACTUATOR_SAIL];}

    ControllerInterface* controller;
		
		static Sailboat* Instance(){if(sailboat == NULL) sailboat = new Sailboat(); return sailboat;}
	private:
		static Sailboat* sailboat;
		
		Sensor* sensors[NB_SENSORS];
		Actuator* actuators[NB_ACTUATORS];
};

#endif
