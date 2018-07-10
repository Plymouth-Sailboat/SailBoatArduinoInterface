#ifndef BATTERY_SENSOR_H
#define BATTERY_SENSOR_H

#include <SensorsInterface.h>
#include <std_msgs/Float32.h>

class BatterySensor : public SensorROS{
public:
    BatterySensor() : SensorROS("battery", &msg, 2000), level(0){}
    void init();
    void updateMeasures();
    void updateTest();
    void communicateData();
    
    double getMeasure(){return level;}
    
private:
    float level;
    std_msgs::Float32 msg;
    
};

#endif

