#include <Sailboat.h>

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
}
void Sailboat::init(){
	sensors[SENSOR_WINDSENSOR] = new WindSensor();
	sensors[SENSOR_GPS] = new GPS();
	sensors[SENSOR_IMU] = new IMU();
	
	actuators[ACTUATOR_RUDDER] = new Rudder();
	actuators[ACTUATOR_SAIL] = new Sail();
	
	for(int i = 0; i < NB_SENSORS; ++i)
		sensors[i]->init();
	
	for(int i = 0; i < NB_ACTUATORS; ++i)
		actuators[i]->init();
}

void Sailboat::updateSensors(){
	for(int i = 0; i < NB_SENSORS; ++i)
		sensors[i]->updateMeasures();
}