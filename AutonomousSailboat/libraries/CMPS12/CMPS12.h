#ifndef CMPS12_SENSOR_H
#define CMPS12_SENSOR_H

#include <IMU.h>
#include <sensor_msgs/Imu.h>

class CMPS12 : public IMU{
	public:
		CMPS12(uint8_t address = 0x60) : IMU("IMU", 10, 10), address(address){}

		void init(ros::NodeHandle* n);
		void updateMeasure();
		void updateTest();

		void storeCalibration();

		float* getQuat(){return quat;}
		float* getAccel(){return accel;}
		float* getMag(){return mag;}
		float* getRot(){return rot;}

	private:
		uint8_t address;
};

#endif
