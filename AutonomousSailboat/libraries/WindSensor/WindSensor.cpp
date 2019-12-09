#include <WindSensor.h>

void WindSensor::init(){
	pinMode(WIND_SENSOR_PIN, INPUT);
#ifdef WIND_ANEMOMETER_PIN
	pinMode(WIND_ANEMOMETER_PIN, INPUT);
#endif
}

void WindSensor::updateAnemometer(){
#ifdef WIND_ANEMOMETER_PIN
	if ((millis() - contactBounceTime) > 15 ) { // debounce the switch contact.
		anemometerRevolution++;
		contactBounceTime = millis();
	}
	if(millis() - timeAnemometer > 2000){
		windSpeed = anemometerRevolution * 1.061/2.0;
		anemometerRevolution = 0;
		timeAnemometer = millis();
	}
#endif
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

	#ifdef WIND_ANEMOMETER_PIN
	if(millis() - timeAnemometer > 4000 && digitalRead(WIND_ANEMOMETER_PIN)==HIGH){
		windSpeed = 0;
		anemometerRevolution=0;
		timeAnemometer = millis();
	}
	#endif
//	Logger::Log(0, F("Wind sensor corrected value :"), String(value));

	// returns the angle, with reference to the boat:
	angle = mapf(value, WIND_SENSOR_MIN, WIND_SENSOR_MAX, WIND_ANGLE_MIN, WIND_ANGLE_MAX);  // The angle is now in the [0;+360] interval
	angle = fmod(angle + 180,360);
    if (angle < 0)
        angle += 360;
  angle -= 180;
	angle = -angle;
	angle = kf.updateEstimate(angle);
	angle = angle*DEG_TO_RAD;
}

void WindSensor::updateTest(){
	angle = 120;
}

void WindSensor::communicateData(){
    msg.x = 0;
    msg.y = 0;
#ifdef WIND_ANEMOMETER_PIN
	msg.x = windSpeed*cos(angle);
	msg.y = windSpeed*sin(angle);
#endif
	msg.theta = angle;

	pub.publish(&msg);
}
