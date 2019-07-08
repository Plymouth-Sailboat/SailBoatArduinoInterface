#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H

#include <SensorsInterface.h>
#include <sensor_msgs/Imu.h>

#include <SimpleKalmanFilter.h>


class IMU : public SensorROS{
	public:
		IMU() : SensorROS("IMU", &msg, 10, 10), wokeUp(false){for(int i = 0; i < 3; ++i)dv[i] = 0;}
		IMU(const char* name, unsigned long period = 10, unsigned long comperiod = 10) : SensorROS(name, &msg, period, comperiod), wokeUp(false){for(int i = 0; i < 3; ++i)dv[i] = 0;}

		virtual void init(ros::NodeHandle* n){}
		virtual void updateMeasures(){updateMeasure(); measureGravity();}
		virtual void updateMeasure() = 0;
		virtual void updateTest() = 0;
		virtual void communicateData() = 0;

		float* getQuat(){return quat;}
		float* getAccel(){return accel;}
		float* getMag(){return mag;}
		float* getRot(){return rot;}
		float* getDv(){return dv;}
		float* getEuler(){return angles;}

		double getHeading(){return angles[0];}
	protected:
		void mulquatvect(float vec[3], float q[4], float res[3]);
		double heading;

		float quat[4];
		float accel[3];
		float mag[3];
		float rot[3];
		float angles[3];
		float dv[3];

		unsigned long timerDv;
		bool wokeUp;

		sensor_msgs::Imu msg;
	private:
		void measureGravity();
};

#endif
