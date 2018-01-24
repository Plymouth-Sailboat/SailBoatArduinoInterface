#ifndef GPS_SENSOR_H
#define GPS_SENSOR_H

#include <SensorsInterface.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <sensor_msgs/NavSatFix.h>
#include <gps_common/GPSFix.h>

class GPS : public SensorROS{
public:
	GPS() : SensorROS("GPS", &msg), ss(GPS_RX, GPS_TX),GPS_latInit(0), GPS_longInit(0), GPS_altInit(0), status(-1), GPS_track(0), GPS_speed(0), time(0), hdop(0), nbSatellites(0){}
	
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
	SoftwareSerial ss;
	double GPS_latInit, GPS_longInit, GPS_altInit;
	double GPS_lat, GPS_long, GPS_alt;
	double GPS_PosX, GPS_PosY;  // Cartesian location of the boat
	double GPS_track, GPS_speed, time, hdop;
	int nbSatellites;
	int status;
	
	gps_common::GPSFix msg;
};

#endif