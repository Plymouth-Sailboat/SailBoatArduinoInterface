#ifndef RC_SENSOR_H
#define RC_SENSOR_H

#include <SensorsInterface.h>

class RC : public Sensor{
public:
	RC(){}
	
	void init();
	void updateMeasures(){}
	void updateTest(){}
	void communicateData(){}
	
private:
};

#endif