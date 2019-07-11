#ifndef GPS_SENSOR_H
#define GPS_SENSOR_H

#include <SensorsInterface.h>
//#include <TinyGPS++.h>
#include <Adafruit_GPS.h>
//#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/String.h>
#include <gps_common/GPSFix.h>
#include <EEPROM.h>
#include <TimeLib.h>
#include <std_msgs/UInt32.h>

class GPS : public SensorROS{
public:
	GPS(HardwareSerial& serial) : SensorROS("GPS/fix", &msg, 50, 500), serial(serial), gps(&Serial1), GPS_latInit(0), GPS_longInit(0), GPS_altInit(0), status(-1), coldStart(false), GPS_track(0), GPS_speed(0), time(0), hdop(0), nbSatellites(0), pubNMEA("GPS/NMEA", &lastNMEA), pubTime("Time", &timeU){}

	void init(ros::NodeHandle* n);
	void updateMeasures();
	void updateTest();
	void communicateData();

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
	Adafruit_GPS gps;
	HardwareSerial& serial;
	double GPS_latInit, GPS_longInit, GPS_altInit;
	double GPS_lat, GPS_long, GPS_alt;
	double GPS_PosX, GPS_PosY;  // Cartesian location of the boat
	double GPS_track, GPS_speed, hdop;
	float lat_std_dev, lon_std_dev, alt_std_dev;
	time_t time;
	uint32_t time_utc;
	int nbSatellites;
	int status;
	bool coldStart;
	uint32_t timerGPS;

	String nmeaD;

	gps_common::GPSFix msg;

	ros::Publisher pubNMEA;
	std_msgs::String lastNMEA;

	ros::Publisher pubTime;
	std_msgs::UInt32 timeU;
};

#endif
