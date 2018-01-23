#ifndef SAILBOAT_H
#define SAILBOAT_H

#include <config.h>

#include <WindSensor.h>
#include <GPS.h>
#include <Xsens.h>

#include <Rudder.h>
#include <Sail.h>

#include <ControllerInterface.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>

class Sailboat{
public:
	Sailboat() : controller(NULL){}
	~Sailboat();
	
	void init(ros::NodeHandle& n);
	void updateSensors();
	void updateTestSensors();
	void communicateData();
	
	WindSensor* getWindSensor(){return (WindSensor*)sensors[SENSOR_WINDSENSOR];}
	GPS* getGPS(){return (GPS*)sensors[SENSOR_GPS];}
	XSens* getIMU(){return (XSens*)sensors[SENSOR_IMU];}
	
	Rudder* getRudder(){return (Rudder*)actuators[ACTUATOR_RUDDER];}
	Sail* getSail(){return (Sail*)actuators[ACTUATOR_SAIL];}
	
	void setController(ControllerInterface* control){controller = control; controller->init();}
    void setController(int index){if(index < nbControllers){controller = controllers[index]; controller->init();}}
	void setControllers(ControllerInterface** control, unsigned int nb){controllers = control; nbControllers = nb;}
	
	void Control();

	void cmdCallback(const geometry_msgs::Twist& msg);
	void msgCallback(const std_msgs::String& msg);
	
	static Sailboat* Instance(){if(sailboat == NULL) sailboat = new Sailboat(); return sailboat;}
private:
	static Sailboat* sailboat;
	
	ControllerInterface** controllers;
	unsigned int nbControllers;
	ControllerInterface* controller;
	Sensor* sensors[NB_SENSORS];
	Actuator* actuators[NB_ACTUATORS];
	
	geometry_msgs::Twist cmd;
	
	int watchdog;
};

#endif
