#ifndef GPS_SUBSCRIBER_SENSOR_H
#define GPS_SUBSCRIBER_SENSOR_H

#include <SensorsInterface.h>

#include <std_msgs/String.h>
#include <gps_common/GPSFix.h>
#include <sensor_msgs/NavSatFix.h>
#include <EEPROM.h>
#include <TimeLib.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/String.h>

#include <Adafruit_GPS.h>

class GPS : public SensorROS{
public:
	GPS() : SensorROS("Test/GPS/fix", &msg, 50, 500), subGPS("GPS/NMEA", &GPS::gps_callback, this), gps(nullptr),GPS_latInit(0), GPS_longInit(0), GPS_altInit(0), status(-1), coldStart(false), GPS_track(0), GPS_speed(0), time(0), hdop(0), nbSatellites(0), GPS_lat(0), GPS_long(0), GPS_alt(0){}

	void init(ros::NodeHandle* n);
	void updateMeasures();
	void updateTest();
	void communicateData();

	void gps_callback(const std_msgs::String& msg);

	double getLat(){return GPS_lat;}
	double getLong(){return GPS_long;}
	double getLatInit(){float latinit = 0.0f; EEPROM.get(0,latinit); return latinit;}
	double getLongInit(){float longinit = 0.0f; EEPROM.get(10,longinit); return longinit;}
	double getX(){return GPS_PosX;}
	double getY(){return GPS_PosY;}
	double getSpeed(){return GPS_speed;}
	double getTrack(){return GPS_track;}
	time_t getTime(){return time;}
	int getStatus(){return status;}
	int getSatellites(){return nbSatellites;}

	void informCold(){coldStart = true;}

private:
	ros::Subscriber<std_msgs::String, GPS> subGPS;
	double GPS_latInit, GPS_longInit, GPS_altInit;
	double GPS_lat, GPS_long, GPS_alt;
	double GPS_PosX, GPS_PosY;  // Cartesian location of the boat
	double GPS_track, GPS_speed, hdop;

	double lat_std_dev, lon_std_dev, alt_std_dev;
	time_t time;
	uint32_t time_utc;
	int nbSatellites;
	int status;
	bool coldStart;
	uint32_t timerGPS;

	std_msgs::String msg;

	std_msgs::UInt32 timeU;

		Adafruit_GPS gps;
};

#endif
