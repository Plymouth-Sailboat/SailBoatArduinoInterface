#ifndef RC_SENSOR_H
#define RC_SENSOR_H

#include <SensorsInterface.h>

class RC : public Sensor{
public:
	RC(){for(int i = 0; i < RC_NUM_CHANNELS; ++i){rc_values[i] = 0; rc_start[i] = 0;}}
	
	void init(){}
	void updateMeasures(){}
	void updateTest(){}
	void communicateData(){}
	
	void interruptCH(uint8_t channel, uint8_t pin);
	uint16_t getRawValue(uint8_t channel){if(channel < RC_NUM_CHANNELS) return rc_values[channel];}
	float getValue(uint8_t channel){if(channel < RC_NUM_CHANNELS) return ((rc_values[channel]-1500.0f)/500.0f);}
	
private:
	uint16_t rc_values[RC_NUM_CHANNELS];
	uint32_t rc_start[RC_NUM_CHANNELS];
};

#endif