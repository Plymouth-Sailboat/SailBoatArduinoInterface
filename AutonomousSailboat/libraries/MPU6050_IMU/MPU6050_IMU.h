#ifndef MPU6050_SENSOR_H
#define MPU6050_SENSOR_H

#include <IMU.h>
#include <sensor_msgs/Imu.h>
#include <MPU6050.h>
#include <I2Cdev.h>

class MPU6050_IMU : public IMU{
	public:
		MPU6050_IMU(uint8_t address = 0x68) : IMU("IMU", 10, 10), address(address), accelgyro(address){}

		void init(ros::NodeHandle* n);
		void updateMeasure();
		void updateTest();

		void startCalibration();
		void stopCalibration();
		void storeCalibration();
	private:
		uint8_t address;
		MPU6050 accelgyro;

		void getCompass_Data();

		volatile float mx_sample[3];
		volatile float my_sample[3];
		volatile float mz_sample[3];

		float mx_centre, my_centre, mz_centre;

		volatile int mx_max =0;
		volatile int my_max =0;
		volatile int mz_max =0;

		volatile int mx_min =0;
		volatile int my_min =0;
		volatile int mz_min =0;

		int16_t ax, ay, az;
		int16_t gx, gy, gz;
		int16_t mx, my, mz;

		I2Cdev   I2C_M;
};

#endif
