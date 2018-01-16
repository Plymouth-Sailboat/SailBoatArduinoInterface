#ifndef GPS_SENSOR_H
#define GPS_SENSOR_H

#define GPS_BAUD_RATE	9600
#define EARTH_RADIUS	6371000  // Earth radius in metres

#include <SensorsInterface.h>
#include <TinyGPS++.h>
#include <sensor_msgs/NavSatFix.h>

class GPS : public Sensor{
public:
	GPS() : Sensor("GPS", &msg){}
	
	void init(ros::NodeHandle& n);
	void updateMeasures();
	void updateTest();
	void communicateData();
	
	double getLat(){return GPS_lat;}
	double getLong(){return GPS_long;}
	double getLatInit(){return GPS_latInit;}
	double getLongInit(){return GPS_longInit;}
	double getX(){return GPS_PosX;}
	double getY(){return GPS_PosY;}
	
private:
	TinyGPSPlus gps;
	double GPS_latInit, GPS_longInit;
	double GPS_lat, GPS_long;
	double GPS_PosX, GPS_PosY;  // Cartesian location of the boat
	
	sensor_msgs::NavSatFix msg;
};

#endif