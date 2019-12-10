#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H

#include <SensorsInterface.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>

#include <SimpleKalmanFilter.h>
#include <GPSIMUFusionV.h>


class IMU : public SensorROS{
	public:
		IMU() : SensorROS("IMU", &msg, 10, 10), pubV("IMU_Dv", &velMsg), wokeUp(false), kfP(0.01,0.01, 0.020)/*, kf(0.005,0.005,0.001), kf1(0.005,0.005,0.001), kf2(0.005,0.005,0.001)*/{for(int i = 0; i < 3; ++i)dv[i] = 0;}
		IMU(const char* name, unsigned long period = 10, unsigned long comperiod = 10) : SensorROS(name, &msg, period, comperiod), pubV("IMU_Dv", &velMsg), wokeUp(false), kfP(0.01,0.01, 0.020)/*, kf(0.05,0.05,0.1), kf1(0.05,0.05,0.1), kf2(0.5,0.5,0.1)*/{for(int i = 0; i < 3; ++i)dv[i] = 0;}

		virtual void init(ros::NodeHandle* n){SensorROS::init(n);n->advertise(pubV); timerFuse = millis();}
		virtual void updateMeasures(){
			updateMeasure();
			measureGravity();
			#ifdef FUSE_GPS_IMU
			if(millis() - timerFuse > 1000){
				fuseGPS_IMU();
				timerFuse = millis();
			}
			#endif
		}
		virtual void updateMeasure() = 0;
		virtual void updateTest() = 0;
		virtual void communicateData();

		virtual void startCalibration(){}
		virtual void stopCalibration(){}
		virtual void storeCalibration(){}
		virtual void startGyroCalibration(){}

		float* getQuat(){return quat;}
		float* getAccel(){return accel;}
		float* getAccelRaw(){return accelRaw;}
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
		float accelRaw[3];
		float mag[3];
		float rot[3];
		float angles[3];
		float dv[3];

		/*SimpleKalmanFilter kf;
		SimpleKalmanFilter kf1;
		SimpleKalmanFilter kf2;*/
		GPSIMUFusionV kfP;

		unsigned long timerDv;
		unsigned long timerFuse;
		bool wokeUp;

		sensor_msgs::Imu msg;
		ros::Publisher pubV;
		geometry_msgs::Twist velMsg;
	private:
		void measureGravity();
		void fuseGPS_IMU();
};

#endif
