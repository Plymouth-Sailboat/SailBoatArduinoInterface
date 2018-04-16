#ifndef SAILBOAT_H
#define SAILBOAT_H

#include <config.h>

#include <WindSensor.h>
#include <GPS.h>
#include <XSens.h>

#include <RCModule.h>

#include <Rudder.h>
#include <Sail.h>

#include <ControllerInterface.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>

class Sailboat{
public:
	Sailboat() : controller(NULL), watchdog(0), watchdogROS(0), timerMillis(0), timerMillisCOM(0), timerMillisCOMAct(0){
		controllerNames[STANDBY_CONTROLLER] = "Standby";
		controllerNames[RUDDERSAIL_CONTROLLER] = "Rudder-Sail";
		controllerNames[RETURNHOME_CONTROLLER] = "Return-Home";
		controllerNames[HEADER_CONTROLLER] = "Heading";
		controllerNames[RC_CONTROLLER] = "RC";
		controllerNames[C_CONTROLLER] = "C";
	}
	~Sailboat();
	
	void init(ros::NodeHandle* n);
	void updateSensors();
	void updateTestSensors();
	void communicateData();
	
	WindSensor* getWindSensor(){return (WindSensor*)sensors[SENSOR_WINDSENSOR];}
	GPS* getGPS(){return (GPS*)sensors[SENSOR_GPS];}
	XSens* getIMU(){return (XSens*)sensors[SENSOR_IMU];}
	
	RC* getRC(){return (RC*)sens[SENSOR_RC];}
	
	Rudder* getRudder(){return (Rudder*)actuators[ACTUATOR_RUDDER];}
	Sail* getSail(){return (Sail*)actuators[ACTUATOR_SAIL];}
	
	void setController(ControllerInterface* control);
    void setController(int index);
	void setControllers(ControllerInterface** control, unsigned int nb){controllers = control; nbControllers = nb;}
	int actualControllerIndex(){return actualControllerI;}
	ControllerInterface* actualController(){return controller;}
	
	void Control();

	void cmdCallback(const geometry_msgs::Twist& msg);
	void msgCallback(const std_msgs::String& msg);
	
	static Sailboat* Instance(){if(sailboat == NULL) sailboat = new Sailboat(); return sailboat;}
private:
	static Sailboat* sailboat;
	
	ControllerInterface** controllers;
	unsigned int nbControllers;
	int actualControllerI;
	ControllerInterface* controller;
	SensorROS* sensors[NB_SENSORS];
	Sensor* sens[NB_SENSORS_NOT_ROS];
	ActuatorROS* actuators[NB_ACTUATORS];
	
	geometry_msgs::Twist cmd;
	
	int watchdog;
	int watchdogROS;
	
	unsigned long timerMillis;
	unsigned long timerMillisCOM;
	unsigned long timerMillisCOMAct;
	
	const char* controllerNames[NB_CONTROLLERS];
};

#endif
