#ifndef SAILBOAT_H
#define SAILBOAT_H

#include <config.h>

#include <WindSensor.h>
#include <GPS.h>
#include <IMU.h>

#include <Rudder.h>
#include <Sail.h>

#include <ControllerInterface.h>
#include <ros.h>
#include <std_msgs/String.h>

class Sailboat{
public:
	Sailboat() : sub("sailboat_msg",&Sailboat::msgCallback, this){}
	~Sailboat();
	
	void init(ros::NodeHandle& n);
	void updateSensors();
	void communicateData();
	
	WindSensor* getWindSensor(){return (WindSensor*)sensors[SENSOR_WINDSENSOR];}
	GPS* getGPS(){return (GPS*)sensors[SENSOR_GPS];}
	IMU* getIMU(){return (IMU*)sensors[SENSOR_IMU];}
	
	Rudder* getRudder(){return (Rudder*)actuators[ACTUATOR_RUDDER];}
	Sail* getSail(){return (Sail*)actuators[ACTUATOR_SAIL];}
	
	void setController(ControllerInterface* control){controller = control; controller->init();}
	void setControllers(ControllerInterface** control, int nb){controllers = control; nbControllers = nb;}
	void Control();

	void msgCallback(const std_msgs::String& msg);
	
	static Sailboat* Instance(){if(sailboat == NULL) sailboat = new Sailboat(); return sailboat;}
private:
	static Sailboat* sailboat;
	
	ControllerInterface** controllers;
	int nbControllers;
	ControllerInterface* controller;
	Sensor* sensors[NB_SENSORS];
	Actuator* actuators[NB_ACTUATORS];
	
	ros::Subscriber<std_msgs::String, Sailboat> sub;
	
	int watchdog;
};

#endif
