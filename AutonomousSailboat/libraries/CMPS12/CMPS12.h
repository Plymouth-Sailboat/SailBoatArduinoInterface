#ifndef CMPS12_SENSOR_H
#define CMPS12_SENSOR_H

#include <IMU.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>

class CMPS12 : public IMU{
	public:
		CMPS12(uint8_t address = 0x60) : IMU("IMU", 10, 10), address(address), wokeUp(false), pubV("IMU_Dv", &velMsg){}
		
		void init(ros::NodeHandle* n);
		void updateMeasures();
		void updateTest();
		void communicateData();
		
		float* getQuat(){return quat;}
		float* getAccel(){return accel;}
		float* getMag(){return mag;}
		float* getRot(){return rot;}
		
	private:
		
		uint8_t address;
		
		bool wokeUp;
		
		ros::Publisher pubV;
		
		geometry_msgs::Twist velMsg;
		
		float quat[4];
		float accel[3];
		float mag[3];
		float rot[3];
		float angles[3];
		float dv[3];
};

#endif