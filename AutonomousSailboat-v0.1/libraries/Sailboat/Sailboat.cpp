#include <Sailboat.h>
#include <TimeLib.h>

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

void Sailboat::msgCallback(const std_msgs::String& msg){
	int msgNB= atoi(msg.data);
	if(msgNB < 1){
		setController(controllers[msgNB]);
	}
}

void Sailboat::init(ros::NodeHandle& n){
	n.subscribe(sub);
	
	sensors[SENSOR_WINDSENSOR] = new WindSensor();
	sensors[SENSOR_GPS] = new GPS();
	sensors[SENSOR_IMU] = new IMU();
	
	actuators[ACTUATOR_RUDDER] = new Rudder();
	actuators[ACTUATOR_SAIL] = new Sail();
	
	for(int i = 0; i < NB_SENSORS; ++i)
		sensors[i]->init(n);
	
	for(int i = 0; i < NB_ACTUATORS; ++i)
		actuators[i]->init();
	
	watchdog = minute();
	if(watchdog > 58)
		watchdog = -1;
}

void Sailboat::updateSensors(){
	for(int i = 0; i < NB_SENSORS; ++i)
		sensors[i]->updateMeasures();
}

void Sailboat::communicateData(){
	for(int i = 0; i < NB_SENSORS; ++i)
		sensors[i]->communicateData();
}


void Sailboat::Control(){
	if(controller != NULL){
		controller->Control();
		
		watchdog = minute();
	}else{
		if(minute() - watchdog > 1)
			controller = controllers[0];
	}
}