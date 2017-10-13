#include <Sailboat.h>

Sailboat::~Sailboat(){
	for(int i = 0; i < NB_SENSORS; ++i){
		if(sensors[i] != NULL)
			delete sensors[i];
	}		
}
void Sailboat::init(){
	sensors[SENSOR_WINDSENSOR] = new WindSensor();
	sensors[SENSOR_GPS] = new GPS();
	
	for(int i = 0; i < NB_SENSORS; ++i)
		sensors[i]->init();
}

void Sailboat::updateSensors(){
	for(int i = 0; i < NB_SENSORS; ++i)
		sensors[i]->updateMeasures();
}