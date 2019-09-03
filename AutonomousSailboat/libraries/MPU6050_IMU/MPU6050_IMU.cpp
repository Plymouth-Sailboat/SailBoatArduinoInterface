#include "MPU6050_IMU.h"
#include <Wire.h>

void MPU6050_IMU::init(ros::NodeHandle* n){
	IMU::init(n);
	accelgyro.initialize();
}

void MPU6050_IMU::updateMeasure(){
  accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
	accel[0] = (double) ax / 16384;
  accel[1] = (double) ay / 16384;
  accel[2] = (double) az / 16384;

	rot[0] = (double) gx * 250 / 32768;
	rot[1] = (double) gy * 250 / 32768;
	rot[2] = (double) gz * 250 / 32768;

	getCompass_Data();
	mag[0] = mag[0] - mx_centre;
	mag[1] = mag[1] - my_centre;
	mag[2] = mag[2] - mz_centre;


  angles[0]=180*atan2(mag[1],mag[0])/M_PI;
  if(angles[0] <0) angles[0] +=360;


  /*float pitch = asin(-accel[0]);
  float roll = asin(accel[1]/cos(pitch));

  float xh = mag[0] * cos(pitch) + mag[2] * sin(pitch);
  float yh = mag[0] * sin(roll) * sin(pitch) + mag[1] * cos(roll) - mag[2] * sin(roll) * cos(pitch);
  float zh = -mag[0] * cos(roll) * sin(pitch) + mag[1] * sin(roll) + mag[2] * cos(roll) * cos(pitch);
  angles[0] = 180 * atan2(yh, xh)/M_PI;
  if(yh<0)    angles[0] +=360;*/
}

void MPU6050_IMU::startCalibration(){
	for (int i=0; i<5000;i++)
	{
		getCompass_Data();
		mx_sample[2] = mag[0];
		my_sample[2] = mag[1];
		mz_sample[2] = mag[2];

		if (mx_sample[2]>=mx_sample[1])mx_sample[1] = mx_sample[2];
		if (my_sample[2]>=my_sample[1])my_sample[1] = my_sample[2]; //find max value
		if (mz_sample[2]>=mz_sample[1])mz_sample[1] = mz_sample[2];

		if (mx_sample[2]<=mx_sample[0])mx_sample[0] = mx_sample[2];
		if (my_sample[2]<=my_sample[0])my_sample[0] = my_sample[2];//find min value
		if (mz_sample[2]<=mz_sample[0])mz_sample[0] = mz_sample[2];
	}

	mx_max = mx_sample[1];
	my_max = my_sample[1];
	mz_max = mz_sample[1];

	mx_min = mx_sample[0];
	my_min = my_sample[0];
	mz_min = mz_sample[0];

	mx_centre = (mx_max + mx_min)/2;
	my_centre = (my_max + my_min)/2;
	mz_centre = (mz_max + mz_min)/2;
}

void MPU6050_IMU::stopCalibration(){

}

void MPU6050_IMU::storeCalibration(){

}

void MPU6050_IMU::updateTest(){
}


void MPU6050_IMU::getCompass_Data()
{
	uint8_t buffer_m[6];
	I2C_M.writeByte(MPU9150_RA_MAG_ADDRESS, 0x0A, 0x01); //enable the magnetometer
	delay(10);
	I2C_M.readBytes(MPU9150_RA_MAG_ADDRESS, MPU9150_RA_MAG_XOUT_L, 6, buffer_m);

  mx = ((int16_t)(buffer_m[1]) << 8) | buffer_m[0] ;
	my = ((int16_t)(buffer_m[3]) << 8) | buffer_m[2] ;
	mz = ((int16_t)(buffer_m[5]) << 8) | buffer_m[4] ;

	mag[0] = (double) mx * 1200 / 4096;
	mag[1] = (double) my * 1200 / 4096;
	mag[2] = (double) mz * 1200 / 4096;
}
