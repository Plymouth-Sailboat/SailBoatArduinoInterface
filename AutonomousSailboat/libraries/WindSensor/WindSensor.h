#ifndef WIND_SENSOR_H
#define WIND_SENSOR_H

#include <SensorsInterface.h>
#include <geometry_msgs/Pose2D.h>

#ifndef HARDWARE_TUNING
#define WIND_SENSOR_OFFSET 198  // May need to be tuned!
                                     // value which gives the zero angle
           							             // Manual tuning does not works, so I chose software tuning
#endif

#define ANGLE_MAX 360
#define ANGLE_MIN 0
			 
class WindSensor : public SensorROS{
	public:
		WindSensor() : SensorROS("wind", &msg), angle(0), windSpeed(0), anemometerRevolution(0), contactBounceTime(0), timeAnemometer(0){}
		void init();
		void updateMeasures();
		void updateTest();
		void communicateData();
		
		void updateAnemometer();
		
		double getMeasure(){return angle;}
		
	private:
		double angle;
		geometry_msgs::Pose2D msg;
		double windSpeed;
		unsigned int anemometerRevolution;
		unsigned long contactBounceTime;
		unsigned long timeAnemometer;
		
};

#endif
