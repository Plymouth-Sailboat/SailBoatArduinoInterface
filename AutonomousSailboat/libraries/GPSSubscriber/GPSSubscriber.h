#ifndef GPS_SUBSCRIBER_SENSOR_H
#define GPS_SUBSCRIBER_SENSOR_H

#include <SensorsInterface.h>

#include <std_msgs/String.h>
#include <gps_common/GPSFix.h>
#include <EEPROM.h>
#include <TimeLib.h>
#include <std_msgs/UInt32.h>

class GPS : public SensorROS{
public:
	GPS() : SensorROS("GPS/fix", &msg, 50, 500), subGPS("GPS/fix", &GPS::gps_callback, this), GPS_latInit(0), GPS_longInit(0), GPS_altInit(0), status(-1), coldStart(false), GPS_track(0), GPS_speed(0), time(0), hdop(0), nbSatellites(0){}

	void init(ros::NodeHandle* n);
	void updateMeasures();
	void updateTest();
	void communicateData();

	void gps_callback(const gps_common::GPSFix& msg);

	double getLat(){return GPS_lat;}
	double getLong(){return GPS_long;}
	double getLatInit(){float latinit = 0.0f; EEPROM.get(0,latinit); return latinit;}
	double getLongInit(){float longinit = 0.0f; EEPROM.get(10,longinit); return longinit;}
	double getX(){return GPS_PosX;}
	double getY(){return GPS_PosY;}
	time_t getTime(){return time;}
	int getStatus(){return status;}
	int getSatellites(){return nbSatellites;}

	void informCold(){coldStart = true;}

private:
	ros::Subscriber<gps_common::GPSFix, GPS> subGPS;
	double GPS_latInit, GPS_longInit, GPS_altInit;
	double GPS_lat, GPS_long, GPS_alt;
	double GPS_PosX, GPS_PosY;  // Cartesian location of the boat
	double GPS_track, GPS_speed, hdop;
	time_t time;
	uint32_t time_utc;
	int nbSatellites;
	int status;
	bool coldStart;
	uint32_t timerGPS;

	gps_common::GPSFix msg;

	std_msgs::UInt32 timeU;
};

#endif
