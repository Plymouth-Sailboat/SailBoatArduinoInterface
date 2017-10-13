#ifndef GPS_SENSOR_H
#define GPS_SENSOR_H

#include <SensorsInterface.h>
#include <TinyGPS++.h>

class GPS : public Sensor{
	public:
		GPS(){}
		
		void init();
		void updateMeasures();
		
	double getLat(){return GPS_lat;}
	double getLong(){return GPS_long;}
	double getX(){return GPS_PosX;}
	double getY(){return GPS_PosY;}
	
	private:
		TinyGPSPlus gps;
		double GPS_latInit, GPS_longInit;
		double GPS_lat, GPS_long;
		double GPS_PosX, GPS_PosY;  // Cartesian location of the boat
};

#endif