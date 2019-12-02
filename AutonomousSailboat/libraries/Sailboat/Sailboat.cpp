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
	if(!((RC*)sens[SENSOR_RC])->controlling ){
		if(controller != NULL)
			controller->setActivated(false);
		if(index < NB_CONTROLLERS){
			actualControllerI = index;
			controller = controllers[index];
			controller->init();
			controller->setActivated(true);

			if(LOGGER)
				Logger::Instance()->Toast("Changed to :", String(controllerNames[index]), 5000);
			publishMsg(String("Changed to :[") + String(index) + "]" + String(controllerNames[index]));
      sailboatmode.data = index;
      pubMode.publish(&sailboatmode);
		}
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
        case 'B':
          if(msg.data[1] == '1'){
            ((IMU*)sensors[SENSOR_IMU])->startCalibration();
            publishMsg(String("Calibration XSens"));
          }
          if(msg.data[1] == '0'){
            ((IMU*)sensors[SENSOR_IMU])->stopCalibration();
            publishMsg(String("Stopped Calibration"));
          }
          if(msg.data[1] == '2'){
            ((IMU*)sensors[SENSOR_IMU])->storeCalibration();
            publishMsg(String("Storing calibration data"));
          }
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

#ifdef USE_ARDUINO_WIND
#pragma message("Using Wind Sensor on Arduino")
    sensors[SENSOR_WINDSENSOR] = new WindSensor();
#else
#pragma message("Attach Wind Sensor on RPI")
    sensors[SENSOR_WINDSENSOR] = new WindSensor();
#endif

#ifdef USE_ARDUINO_GPS
#pragma message("Using GPS on Arduino")
	if(GPS_SERIAL == 1)
		sensors[SENSOR_GPS] = new GPS(Serial1);
	if(GPS_SERIAL == 2)
		sensors[SENSOR_GPS] = new GPS(Serial2);
  if(GPS_SERIAL == 3)
    sensors[SENSOR_GPS] = new GPS(Serial3);
#else
#pragma message("Attach GPS on RPI")
  sensors[SENSOR_GPS] = new GPS();
#endif

#if defined(XSENS_IMU)
	#pragma message("XSENS is used as IMU")
  sensors[SENSOR_IMU] = new XSens();
#elif defined(CMPS12_IMU)
	#pragma message("CMPS12 is used as IMU")
	sensors[SENSOR_IMU] = new CMPS12();
#elif defined(JY901_IMU)
	#pragma message("JY901 is used as IMU")
	sensors[SENSOR_IMU] = new JY901IMU();
#endif
  sensors[SENSOR_BATTERY] = new BatterySensor();

  sens[SENSOR_RC] = new RC();
#ifdef SERVO_SHIELD
	#pragma message("Servo Shield from Adafruit is used")
	Adafruit_PWMServoDriver* servo_motors_pwm = new Adafruit_PWMServoDriver(&Wire, SERVO_ADDRESS);
    actuators[ACTUATOR_RUDDER] = new Servo_Motor(RUDDER_SERVO,RUDDER_POS_NEUTRAL,RUDDER_POS_MAX,RUDDER_POS_MIN,RUDDER_MIN,RUDDER_MAX,"rudder");
	((Servo_Motor*)actuators[ACTUATOR_RUDDER])->setMotor(servo_motors_pwm);
    actuators[ACTUATOR_SAIL] = new Servo_Motor(WINCH_SERVO,WINCH_ANGLE_NEUTRAL,WINCH_ANGLE_MAX, WINCH_ANGLE_MIN,SAIL_MIN,SAIL_MAX,"sail");
	((Servo_Motor*)actuators[ACTUATOR_SAIL])->setMotor(servo_motors_pwm);
#ifdef ACTUATOR_RUDDER2
    actuators[ACTUATOR_RUDDER2] = new Servo_Motor(RUDDER2_SERVO, RUDDER2_POS_NEUTRAL, RUDDER2_POS_MAX, RUDDER2_POS_MIN, RUDDER2_MIN,RUDDER2_MAX,"rudder2");
	((Servo_Motor*)actuators[ACTUATOR_RUDDER2])->setMotor(servo_motors_pwm);
#endif
#else
	#pragma message("Custom interface board is used")
    actuators[ACTUATOR_RUDDER] = new Servo_Motor(RUDDER_PIN,RUDDER_POS_NEUTRAL,RUDDER_POS_MAX,RUDDER_POS_MIN,RUDDER_MIN,RUDDER_MAX,"rudder");
    actuators[ACTUATOR_SAIL] = new Servo_Motor(WINCH_PIN,WINCH_ANGLE_NEUTRAL,WINCH_ANGLE_MAX, WINCH_ANGLE_MIN,SAIL_MIN,SAIL_MAX,"sail");
#ifdef ACTUATOR_RUDDER2
    actuators[ACTUATOR_RUDDER2] = new Servo_Motor(RUDDER2_PIN, RUDDER2_POS_NEUTRAL, RUDDER2_POS_MAX, RUDDER2_POS_MIN, RUDDER2_MIN,RUDDER2_MAX,"rudder2");
#endif
#endif

    for(int i = 0; i < NB_SENSORS; ++i)
        sensors[i]->init(n);

    for(int i = 0; i < NB_ACTUATORS; ++i)
        actuators[i]->init(n);

    watchdog = millis();
    watchdogROS = millis();

    n->advertise(pubMsg);
    n->advertise(pubMode);
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
