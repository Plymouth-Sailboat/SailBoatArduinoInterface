#ifndef XSENS_SENSOR_H
#define XSENS_SENSOR_H

#include <IMU.h>
#include <sensor_msgs/Imu.h>
#include "bus/XBus.h"

class XSens : public IMU{
	public:
		XSens(uint8_t address = 0x1d) : IMU("IMU", 10, 10), address(address), xbus(address){}

		void init(ros::NodeHandle* n);
		void updateMeasure();
		void updateTest();

		void startCalibration();
		void stopCalibration();
		void storeCalibration();
		void startGyroCalibration();

		void setGPSPosition(float lat, float longitude, float alt);
	private:

		uint8_t address;

		XBus xbus;
};

#endif
