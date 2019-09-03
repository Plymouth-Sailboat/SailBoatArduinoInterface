#include <GPSSubscriber.h>
#include    <stdlib.h>

void GPS::init(ros::NodeHandle* n){
	n->subscribe(subGPS);
	//n->advertise(pub);
}

void GPS::updateMeasures(){

}

void GPS::updateTest(){
	GPS_lat = 50.365595;
	GPS_long = -4.143274;
}

void GPS::communicateData(){
	//String ms = "working";
	//char buf[100];
	//ms+=String(GPS_lat);
	//String(ms).toCharArray(buf,100);
	//test.data = buf;
	//pub.publish(&test);
}

void GPS::gps_callback(const gps_common::GPSFix& msg){
		GPS_lat=msg.latitude;
		GPS_long=msg.longitude;
		GPS_alt=msg.altitude;

		GPS_track = msg.track;
		GPS_speed = msg.speed;
		time_utc = msg.time;
		hdop = msg.hdop;

		nbSatellites = msg.status.satellites_used;
		nbSatellites = msg.status.satellites_visible;
		/////********COULD BUILD SATELLITES INFO******////
		status = msg.status.status;
		//msg.status.motion_source;
		//msg.status.orientation_source;
		//msg.status.position_source;

		//msg.position_covariance_type;
		if(status == 0){
			if(GPS_altInit == 0.0f)
				GPS_altInit = GPS_alt;
			if(GPS_latInit == 0.0f){
				GPS_latInit = GPS_lat;
				if(coldStart)
					EEPROM.put(0,GPS_lat);
			}
			if(GPS_longInit == 0.0f){
				GPS_longInit = GPS_long;
				if(coldStart){
					EEPROM.put(10,GPS_long);
					coldStart = false;
				}
			}
		}
}
