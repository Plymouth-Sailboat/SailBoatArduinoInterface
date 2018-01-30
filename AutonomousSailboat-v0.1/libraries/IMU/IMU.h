#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H

// Accelerometer/Gyro register addresses
#define ACCEL_CONFIG 0x1C
#define GYRO_CONFIG 0x1B
#define ACCEL_XOUT_H 0x3B
#define ACCEL_YOUT_H 0x3D
#define ACCEL_ZOUT_H 0x3F
#define TEMP_OUT_H 0x41
#define GYRO_XOUT_H 0x43
#define GYRO_YOUT_H 0x45
#define GYRO_ZOUT_H 0x47
#define PWR_MGMT_1 0x6B

//Compass register addresses
#define COMP_STATUS 0x02
#define COMP_XOUT_L 0x03
#define COMP_YOUT_L 0x05
#define COMP_ZOUT_L 0x07

// Accelerometer range modes
#define ACCELRANGE_2g 0
#define ACCELRANGE_4g 1
#define ACCELRANGE_8g 2
#define ACCELRANGE_16g 3

// Gyroscope sensitivity
#define GYRORANGE_250DPS 0
#define GYRORANGE_500DPS 1
#define GYRORANGE_1000DPS 2
#define GYRORANGE_2000DPS 3
#define M_CENTER_X 0.0978 // translation vector - Compass calibration
#define M_CENTER_Y 47.2903
#define M_CENTER_Z 16.4626

#define CC11 0.9960  // Homothetic Matrix - Compass calibration
#define CC12 0.0014
#define CC13 -0.0124
#define CC21 0.0014
#define CC22 0.9719
#define CC23 0.0029
#define CC31 -0.0124
#define CC32 0.0029
#define CC33 0.9612
     
#define C0 1.0  // Coefficients of the moving average filter
#define C1 1.0
#define C2 1.0
			 
#include <SensorsInterface.h>
#include <sensor_msgs/Imu.h>

typedef struct IMUdata {
  float x,  // coordinate x
        y,  // coordinate y
        z;  // coordinate z
} IMUData;

typedef struct IMUstruct {
  IMUData Accelero,  // Accelerometer data
          Gyro,  // Gyroscope data
          Magneto;  // Magnetometer data

} IMUStruct;


class IMU : public SensorROS{
	public:
		IMU() : SensorROS("IMU", &msg){}
		
		void init(ros::NodeHandle* n);
		void updateMeasures();
		void updateTest();
		void communicateData();
		
		double getHeading(){return heading;}
	private:
		IMUStruct IMUs;
		double heading;
		double heading1;
		double heading2;
	
		int Read_Compass(byte axis);
		void Trigger_Compass();
		double findHeading(int compass_x, int compass_y);
		
		sensor_msgs::Imu msg;
};

#endif