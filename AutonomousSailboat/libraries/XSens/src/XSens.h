#ifndef XSENS_SENSOR_H
#define XSENS_SENSOR_H

#include <SensorsInterface.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include "bus/XBus.h"

class XSens : public SensorROS{
	public:
		XSens(uint8_t address = 0x1d) : SensorROS("IMU", &msg, 10, 10), address(address), xbus(address), wokeUp(false), pubV("IMU_Dv", &velMsg){}
		
		void init(ros::NodeHandle* n);
		void updateMeasures();
		void updateTest();
		void communicateData();
		
		float getHeadingYaw(){return xbus.headingYaw;}
		void setGPSPosition(float lat, float longitude, float alt);
		
		float* getQuat(){return xbus.quat;}
		float* getAccel(){return xbus.accel;}
		float* getMag(){return xbus.mag;}
		float* getRot(){return xbus.rot;}
		
	private:
		
		uint8_t address;
		
		bool wokeUp;
		
		ros::Publisher pubV;
		
		sensor_msgs::Imu msg;
		geometry_msgs::Twist velMsg;
		XBus xbus;
};

#endif