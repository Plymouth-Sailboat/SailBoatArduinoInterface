#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H
			 
#include <SensorsInterface.h>
#include <sensor_msgs/Imu.h>


class IMU : public SensorROS{
	public:
		IMU() : SensorROS("IMU", &msg, 10, 10){}
		IMU(const char* name, unsigned long period = 10, unsigned long comperiod = 10) : SensorROS(name, &msg, period, comperiod){}
		
		virtual void init(ros::NodeHandle* n){};
		void updateMeasures() = 0;
		void updateTest() = 0;
		void communicateData() = 0;
		
		double getHeading(){return heading;}
	protected:
		double heading;
		
		sensor_msgs::Imu msg;
};

#endif