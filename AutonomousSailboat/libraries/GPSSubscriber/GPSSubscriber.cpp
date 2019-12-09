#include <GPSSubscriber.h>
#include    <stdlib.h>

void GPS::init(ros::NodeHandle* n){
	//SensorROS::init(n);
	nh = n;
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
}

void GPS::gps_callback(const std_msgs::String& msg){
	char* buf =	strdup(msg.data);
	gps.parse(buf);

	status = (int)gps.fix-1;
	timeU.data = time;
	time_utc = ((uint32_t)gps.hour*10000)+(uint32_t)gps.minute*100+(uint32_t)gps.seconds;
	if (gps.fix) {
		setTime((int)gps.hour, (int)gps.minute, (int)gps.seconds, (int)gps.day, (int)gps.month, (int)gps.year);

		hdop = gps.HDOP;

		GPS_lat = gps.latitudeDegrees;
		GPS_long = gps.longitudeDegrees;
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
		GPS_speed = gps.speed;
		GPS_track = gps.angle*M_PI/180.0;
		GPS_alt = gps.altitude;
		lat_std_dev = gps.lat_std_dev;
		lon_std_dev = gps.lon_std_dev;
		alt_std_dev = gps.alt_std_dev;
		nbSatellites = (int)gps.satellites;
	}
	free(buf);
}
