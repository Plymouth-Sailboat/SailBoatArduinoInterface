#ifndef WIND_SUBSCRIBER_SENSOR_H
#define WIND_SUBSCRIBER_SENSOR_H

#include <SensorsInterface.h>

#include <geometry_msgs/Pose2D.h>

class WindSensor : public SensorROS{
public:
	WindSensor() : SensorROS("Test/wind", &msgTest, 50, 500)/*, pub("test", &test)*/, subWind("wind", &WindSensor::wind_callback, this), angle(0), windSpeed(0){}

	void init(ros::NodeHandle* n);
	void updateMeasures();
	void updateTest();
	void communicateData();

	void wind_callback(const geometry_msgs::Pose2D& msg);

	void updateAnemometer();

	double getMeasure(){return angle;}
	double getSpeed(){return windSpeed;}

private:
	ros::Subscriber<geometry_msgs::Pose2D, WindSensor> subWind;
	double angle;
	geometry_msgs::Pose2D msgTest;
	double windSpeed;
};

#endif
