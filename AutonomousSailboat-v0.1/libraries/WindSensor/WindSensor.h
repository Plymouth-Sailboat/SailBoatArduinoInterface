#ifndef WIND_SENSOR_H
#define WIND_SENSOR_H

#include <SensorsInterface.h>

#define WIND_SENSOR_MIN 51  // Corresponds approximately at 5% of the maximum readable (1023) 
#define WIND_SENSOR_MAX 972  // Corresponds approximately at 95% of the minimum readable (1023)

#ifndef HARDWARE_TUNING
#define WIND_SENSOR_OFFSET 198  // May need to be tuned!
                                     // value which gives the zero angle
           							             // Manual tuning does not works, so I chose software tuning
#endif

#define ANGLE_MAX 360
#define ANGLE_MIN 0
			 
class WindSensor : public Sensor{
	public:
		WindSensor(){}
		
		void init();
		void updateMeasures();
		
		double getMeasure(){return angle;}
		
	private:
		double angle;
};

#endif