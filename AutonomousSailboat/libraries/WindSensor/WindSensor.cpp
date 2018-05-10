#include <WindSensor.h>

void WindSensor::init(){
	pinMode(WIND_SENSOR_PIN, INPUT);
#ifdef WIND_ANEMOMETER_PIN
	pinMode(WIND_ANEMOMETER_PIN, INPUT); 
#endif
}

void WindSensor::updateAnemometer(){
	if ((millis() - contactBounceTime) > 15 ) { // debounce the switch contact. 
		anemometerRevolution++; 
		contactBounceTime = millis(); 
	}
	if(millis() - timeAnemometer > 3000){
		windSpeed = anemometerRevolution * 0.3354;
		anemometerRevolution = 0;
		timeAnemometer = millis();
	}
}

void WindSensor::updateMeasures(){
//	Logger::Log(1, F("SetupWindSensor()"), F(""));  // Done in the Setup
	// Safety:

	value = analogRead(WIND_SENSOR_PIN);
	// If no value / no sensor:
	if ((value < WIND_SENSOR_MIN) || (value > WIND_SENSOR_MAX)){
		// This test might not detect if the signal wire is not linked but if it is activated, 
		//   there is 100% chances that it's true !
//		Logger::Warning(F("WindAngle"), F("No Wind Sensor/Value read or sensor in bad state!"));
	}
	else {
		// Feedback initialization:
//		Logger::Message(F("##\t"), F("Wind Sensor seems to be OK"), F(""), 1);
	}
	
//	Logger::Log(0, F("Wind sensor corrected value :"), String(value));

	// returns the angle, with reference to the boat:
	angle = mapf(value, WIND_SENSOR_MIN, WIND_SENSOR_MAX, ANGLE_MIN, ANGLE_MAX);  // The angle is now in the [0;+360] interval

	// To set the angle in the [-180;+180] interval
	if (angle > 180) {
		angle -= ANGLE_MAX;
	}
	angle = -angle;
}

void WindSensor::updateTest(){
	angle = 120;
}

void WindSensor::communicateData(){
	msg.x = windSpeed*cos(angle*DEG_TO_RAD);
	msg.y = windSpeed*sin(angle*DEG_TO_RAD);
	msg.theta = angle*DEG_TO_RAD;
	
	pub.publish(&msg);
}
