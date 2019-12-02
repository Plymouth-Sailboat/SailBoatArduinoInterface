#ifndef SAILBOAT_H
#define SAILBOAT_H

#include <config-Sailboat.h>

#ifdef USE_ARDUINO_WIND
#include <WindSensor.h>
#else
#include <WindSensorSubscriber.h>
#endif

#ifdef USE_ARDUINO_GPS
#include <GPS.h>
#else
#include <GPSSubscriber.h>
#endif

#include <XSens.h>
#include <CMPS12.h>
#include <JY901IMU.h>
#include <BatterySensor.h>

#include <RCModule.h>

#include <Servo_Motor.h>
//#include <Rudder.h>
//#include <Sail.h>

#include <ControllerInterface.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>

class Sailboat{
public:
	Sailboat() : controller(NULL), pubMsg("sailboat_log", &sailboatmsgs), pubMode("sailboat_mode", &sailboatmode), watchdog(0), watchdogROS(0), timerMillis(0), timerMillisCOM(0), timerMillisCOMAct(0){
		controllerNames[STANDBY_CONTROLLER] = "Standby";
		controllerNames[RUDDERSAIL_CONTROLLER] = "Rudder-Sail";
		controllerNames[RETURNHOME_CONTROLLER] = "Return-Home";
		controllerNames[HEADER_CONTROLLER] = "Heading";
		controllerNames[RC_CONTROLLER] = "RC";
		controllerNames[SAILCAP_CONTROLLER] = "Sail-Cap";
		controllerNames[RUDDER_CONTROLLER] = "Rudder";
	}
	~Sailboat();

	void init(ros::NodeHandle* n);
	void updateSensors();
	void updateTestSensors();
	void communicateData();

	WindSensor* getWindSensor(){return (WindSensor*)sensors[SENSOR_WINDSENSOR];}
	GPS* getGPS(){return (GPS*)sensors[SENSOR_GPS];}
	IMU* getIMU(){return (XSens*)sensors[SENSOR_IMU];}
  BatterySensor* getBattery(){return (BatterySensor*)sensors[SENSOR_BATTERY];}

	RC* getRC(){return (RC*)sens[SENSOR_RC];}

	Servo_Motor* getRudder(){return (Servo_Motor*)actuators[ACTUATOR_RUDDER];}
	Servo_Motor* getSail(){return (Servo_Motor*)actuators[ACTUATOR_SAIL];}
#ifdef ACTUATOR_RUDDER2
	Servo_Motor* getRudder2(){return (Servo_Motor*)actuators[ACTUATOR_RUDDER2];}
#endif

	void setController(ControllerInterface* control);
  void setController(int index);
	void setControllers(ControllerInterface** control){controllers = control;}
	int actualControllerIndex(){return actualControllerI;}
	ControllerInterface* actualController(){return controller;}

	void Control();

	void cmdCallback(const geometry_msgs::Twist& msg);
	void msgCallback(const std_msgs::String& msg);

  void publishMsg(String msg);
  void publishMsg(const char* msg);

  void resetWatchdogROS(){watchdogROS = millis();}

	static Sailboat* Instance(){if(sailboat == NULL) sailboat = new Sailboat(); return sailboat;}
private:
	static Sailboat* sailboat;

	ControllerInterface** controllers;
	int actualControllerI;
	ControllerInterface* controller;
	SensorROS* sensors[NB_SENSORS];
	Sensor* sens[NB_SENSORS_NOT_ROS];
	ActuatorROS* actuators[NB_ACTUATORS];

	geometry_msgs::Twist cmd;

  ros::Publisher pubMsg;
  ros::Publisher pubMode;
  std_msgs::String sailboatmsgs;
  std_msgs::UInt32 sailboatmode;

	unsigned long watchdog;
	unsigned long watchdogROS;

	unsigned long timerMillis;
	unsigned long timerMillisCOM;
	unsigned long timerMillisCOMAct;

	const char* controllerNames[NB_CONTROLLERS];
};

#endif
