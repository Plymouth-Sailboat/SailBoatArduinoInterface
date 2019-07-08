#include <GPSSubscriber.h>

void GPS::init(ros::NodeHandle* n){
	n->subscribe(subGPS);
}

void GPS::updateMeasures(){

}

void GPS::updateTest(){
	GPS_lat = 50.365595;
	GPS_long = -4.143274;
}

void GPS::communicateData(){
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

		msg.header.stamp = nh->now();
}
