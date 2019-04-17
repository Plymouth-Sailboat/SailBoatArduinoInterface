#include <Sailboat.h>
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
    
    for(int i = 0; i < NB_CONTROLLERS; ++i){
        if(controllers[i] != NULL)
            delete controllers[i];
    }
}

void Sailboat::setController(ControllerInterface* control){
    if(controller != NULL)
        controller->setActivated(false);
    controller = control;
    controller->init();
    controller->setActivated(true);
    
    ((RC*)sens[SENSOR_RC])->controlling = false;
    
    actualControllerI = -1;
}
void Sailboat::setController(int index){
    if(controller != NULL)
        controller->setActivated(false);
    if(index < NB_CONTROLLERS){
        actualControllerI = index;
        controller = controllers[index];
        controller->init();
        controller->setActivated(true);
        
        ((RC*)sens[SENSOR_RC])->controlling = false;
        
        if(LOGGER)
            Logger::Instance()->Toast("Changed to :", String(controllerNames[index]), 5000);
        publishMsg(String("Changed to :") + String(controllerNames[index]));
    }
}

void Sailboat::cmdCallback(const geometry_msgs::Twist& msg){
    cmd = msg;
}

void Sailboat::msgCallback(const std_msgs::String& msg){
    switch(msg.data[0]){
        case 'C':
            setController(msg.data[1] - '0');
            break;
        case 'M':
            break;
        case 'P':
            if(LOGGER)
                Logger::Instance()->Toast("From PC : ", String(msg.data+1), 5000);
            publishMsg(String("From PC : ") + String(msg.data+1));
            break;
    }
    watchdogROS = millis();
}

void Sailboat::init(ros::NodeHandle* n){
    Wire.begin();
    
    sensors[SENSOR_WINDSENSOR] = new WindSensor();
	if(GPS_SERIAL == 1)
		sensors[SENSOR_GPS] = new GPS(Serial1);
	if(GPS_SERIAL == 2)
		sensors[SENSOR_GPS] = new GPS(Serial2);
    if(GPS_SERIAL == 3)
        sensors[SENSOR_GPS] = new GPS(Serial3);
    sensors[SENSOR_IMU] = new XSens();
    sensors[SENSOR_BATTERY] = new BatterySensor();
    
    sens[SENSOR_RC] = new RC();
    
    actuators[ACTUATOR_RUDDER] = new Servo_Motor(RUDDER_PIN,RUDDER_POS_NEUTRAL,RUDDER_POS_MAX,RUDDER_POS_MIN,RUDDER_MIN,RUDDER_MAX,"rudder");
    actuators[ACTUATOR_SAIL] = new Servo_Motor(WINCH_PIN,WINCH_ANGLE_NEUTRAL,WINCH_ANGLE_MAX, WINCH_ANGLE_MIN,SAIL_MIN,SAIL_MAX,"sail");
#ifdef ACTUATOR_RUDDER2
    actuators[ACTUATOR_RUDDER2] = new Servo_Motor(RUDDER2_PIN, RUDDER2_POS_NEUTRAL, RUDDER2_POS_MAX, RUDDER2_POS_MIN, RUDDER2_MIN,RUDDER2_MAX,"rudder2");
#endif
    
    for(int i = 0; i < NB_SENSORS; ++i)
        sensors[i]->init(n);
    
    for(int i = 0; i < NB_ACTUATORS; ++i)
        actuators[i]->init(n);
    
    watchdog = millis();
    watchdogROS = millis();
    
    n->advertise(pubMsg);
}

void Sailboat::publishMsg(String msg){
    char buf[256];
    msg.toCharArray(buf,256);
    sailboatmsgs.data = buf;
    pubMsg.publish(&sailboatmsgs);
}

void Sailboat::publishMsg(const char* msg){
    sailboatmsgs.data = msg;
    pubMsg.publish(&sailboatmsgs);
}

void Sailboat::updateSensors(){
    for(int i = 0; i < NB_SENSORS; ++i){
        sensors[i]->update();
    }
    
    for(int i = 0; i < NB_SENSORS_NOT_ROS; ++i){
        sens[i]->update();
    }
}

void Sailboat::updateTestSensors(){
    for(int i = 0; i < NB_SENSORS; ++i)
        sensors[i]->updateT();
}

void Sailboat::communicateData(){
    if(millis() - timerMillisCOM > 2){
        for(int i = 0; i < NB_SENSORS; ++i)
            sensors[i]->communicate();
        timerMillisCOM = millis();
    }
    if(millis() - timerMillisCOMAct > 10){
        for(int i = 0; i < NB_ACTUATORS; ++i)
            actuators[i]->communicateData();
        timerMillisCOMAct = millis();
    }
}


void Sailboat::Control(){
    if(millis() - timerMillis > 100){
        if(controller != nullptr){
            controller->ControlTime(cmd);
            watchdog = millis();
        }else{
            if(millis() - watchdog > 60000)
                setController(RETURNHOME_CONTROLLER);
        }
        
        if(millis() - watchdogROS > 300000){
            if(LOGGER)
                Logger::Instance()->Toast("ROS DEAD??", "ROS DEAD??", 0);
            publishMsg("ROS DEAD??");
			if(controller != controllers[RC_CONTROLLER])
				setController(RETURNHOME_CONTROLLER);
        }
        
        for(int i =0; i < NB_CONTROLLERS; ++i){
            if(controllers[i] != NULL && !controllers[i]->isActivated()){
                controllers[i]->updateBackground();
            }
        }
    }
}

