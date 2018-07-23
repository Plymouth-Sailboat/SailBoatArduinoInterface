#ifndef RC_SENSOR_H
#define RC_SENSOR_H

#include <SensorsInterface.h>
#include <Helper.h>

class RC : public Sensor{
public:
	RC() : controlling(false), timer(0), previousController(RETURNHOME_CONTROLLER){
		for(int i = 0; i < 6; ++i){
			rc_values[i] = 0; rc_start[i] = 0;
		}
		offsetmin[RC_1] = RC_1_MIN;
		offsetmax[RC_1] = RC_1_MAX;
		offsetmin[RC_2] = RC_2_MIN;
		offsetmax[RC_2] = RC_2_MAX;
		offsetmin[RC_3] = RC_3_MIN;
		offsetmax[RC_3] = RC_3_MAX;
		offsetmin[RC_4] = RC_4_MIN;
		offsetmax[RC_4] = RC_4_MAX;
		offsetmin[RC_5] = RC_5_MIN;
		offsetmax[RC_5] = RC_5_MAX;
		offsetmin[RC_6] = RC_6_MIN;
		offsetmax[RC_6] = RC_6_MAX;
		}
		
	void init(){
	}
	void updateMeasures();
	void updateTest(){}
	void communicateData(){}
	
	void interruptCH(uint8_t channel, uint8_t pin);
	uint16_t getRawValue(uint8_t channel){if(channel < RC_NUM_CHANNELS) return rc_values[channel];}
	float getValue(uint8_t channel){if(channel < RC_NUM_CHANNELS) return mapf(rc_values[channel], offsetmin[channel], offsetmax[channel], 0.0, 1.0);}
	bool controlling;
	
private:
	uint16_t rc_values[6];
	uint32_t rc_start[6];
		
	unsigned long timer;
	unsigned long watchdog;
	unsigned int offsetmin[6];
	unsigned int offsetmax[6];
	
	int previousController;
};

#endif