#include <BatterySensor.h>

void BatterySensor::init(){
	pinMode(BATTERY_SENSOR_PIN, INPUT);
}

void BatterySensor::updateMeasures(){
	value = analogRead(BATTERY_SENSOR_PIN);
    level = value*0.01303356142; // 1/1023.0 * 5.0 * (12000.0+20000.0)/12000.0
}

void BatterySensor::updateTest(){
	level = 6.0;
}

void BatterySensor::communicateData(){
	msg.data = level;
	pub.publish(&msg);
}
