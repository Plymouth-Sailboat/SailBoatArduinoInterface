#include <fs-r6b.h>
#include <Arduino.h>

void RC::interruptCH(uint8_t channel, uint8_t pin){
	if (digitalRead(pin) == HIGH) {
		rc_start[channel] = micros();
	} else {
		uint16_t rc_compare = (uint16_t)(micros() - rc_start[channel]);
		rc_values[channel] = rc_compare;
	}
}