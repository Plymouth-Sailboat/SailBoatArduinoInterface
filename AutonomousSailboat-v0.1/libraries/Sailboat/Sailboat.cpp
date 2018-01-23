#include <Sailboat.h>
#include <TimeLib.h>
#include <Wire.h>

Sailboat* Sailboat::sailboat = NULL;

Sailboat::~Sailboat(){
	for(int i = 0; i < NB_SENSORS; ++i){
		if(sensors[i] != NULL)
			delete sensors[i];
	}
	
	for(int i = 0; i < NB_ACTUATORS; ++i){
		if(actuators[i] != NULL)
			delete actuators[i];
	}
	
	for(int i = 0; i < nbControllers; ++i){
		if(controllers[i] != NULL)
			delete controllers[i];
	}
}

void Sailboat::cmdCallback(const geometry_msgs::Twist& msg){
	cmd = msg;
}

void Sailboat::msgCallback(const std_msgs::String& msg){
	Logger::Instance()->Log(0, "Received", msg.data);
	switch(msg.data[0]){
	case 'C':
		setController(msg.data[1] - '0');
		break;
	case 'M':
		break;
	} 
	watchdogROS = minute();
}

void Sailboat::init(ros::NodeHandle& n){
	Wire.begin();
	
	sensors[SENSOR_WINDSENSOR] = new WindSensor();
	sensors[SENSOR_GPS] = new GPS();
	sensors[SENSOR_IMU] = new XSens();
	
	actuators[ACTUATOR_RUDDER] = new Rudder();
	actuators[ACTUATOR_SAIL] = new Sail();
	
	for(int i = 0; i < NB_SENSORS; ++i)
		sensors[i]->init(n);
	
	for(int i = 0; i < NB_ACTUATORS; ++i)
		actuators[i]->init();
	
	watchdog = minute();
	watchdogROS = minute();
	if(watchdog > 58)
		watchdog = -1;
	if(watchdogROS > 58)
		watchdogROS = -1;
}

void Sailboat::updateSensors(){
	for(int i = 0; i < NB_SENSORS; ++i)
	sensors[i]->update();
}

void Sailboat::updateTestSensors(){
	for(int i = 0; i < NB_SENSORS; ++i)
	sensors[i]->updateT();
}

void Sailboat::communicateData(){
	if(millis() - timerMillisCOM > 20){
		for(int i = 0; i < NB_SENSORS; ++i)
			sensors[i]->communicateData();
	}
}


void Sailboat::Control(){
	if(millis() - timerMillis > 100){
		if(controller != NULL){
			controller->Control(cmd);
			watchdog = minute();
		}else{
			if(minute() - watchdog > 1)
				setController(0);
		}
		
		if(minute() - watchdogROS > 5){
			setController(2);
		}
	}
}