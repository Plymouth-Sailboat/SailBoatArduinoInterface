#include <RCModule.h>
#include <Arduino.h>
#include <Sailboat.h>

void RC::updateMeasures(){
	if(millis() - watchdog > 10000 && controlling){
		controlling = false;
		Sailboat::Instance()->setController(previousController);
	}
}

void RC::interruptCH(uint8_t channel, uint8_t pin){
	if (digitalRead(pin) == HIGH) {
		rc_start[channel] = micros();
	} else {
		uint16_t rc_compare = (uint16_t)(micros() - rc_start[channel]);
		rc_values[channel] = rc_compare;
	}
	if(!controlling){
		previousController = Sailboat::Instance()->actualControllerIndex();
		Sailboat::Instance()->setController(RC_CONTROLLER);
	}
	controlling = true;
	watchdog = millis();
}