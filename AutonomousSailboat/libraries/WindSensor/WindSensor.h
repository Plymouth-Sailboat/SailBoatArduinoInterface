#ifndef WIND_SENSOR_H
#define WIND_SENSOR_H

#include <SensorsInterface.h>
#include <geometry_msgs/Pose2D.h>
#include <SimpleKalmanFilter.h>


class WindSensor : public SensorROS{
	public:
		WindSensor() : SensorROS("wind", &msg), angle(0), windSpeed(0), anemometerRevolution(0), contactBounceTime(0), timeAnemometer(0), kf(5,5,0.08){}
		void init();
		void updateMeasures();
		void updateTest();
		void communicateData();

		void updateAnemometer();

		double getMeasure(){return angle;}
		double getSpeed(){return windSpeed;}

	private:
		double angle;
		geometry_msgs::Pose2D msg;
		double windSpeed;
		unsigned int anemometerRevolution;
		unsigned long contactBounceTime;
		unsigned long timeAnemometer;

		SimpleKalmanFilter kf;

};

#endif
