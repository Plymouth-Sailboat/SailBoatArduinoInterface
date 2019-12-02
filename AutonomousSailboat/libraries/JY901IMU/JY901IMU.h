#ifndef JY901IMU_SENSOR_H
#define JY901IMU_SENSOR_H

#include <IMU.h>
#include <sensor_msgs/Imu.h>

class JY901IMU : public IMU{
	public:
		JY901IMU(uint8_t address = 0x50) : IMU("IMU", 10, 10), address(address){}

		void init(ros::NodeHandle* n);
		void updateMeasure();
		void updateTest();

		void startCalibration();
		void stopCalibration();
		void storeCalibration();

		float* getQuat(){return quat;}
		float* getAccel(){return accel;}
		float* getMag(){return mag;}
		float* getRot(){return rot;}

	private:
		uint8_t address;
};

#endif
