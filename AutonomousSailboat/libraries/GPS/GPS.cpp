#include <GPS.h>

void GPS::init(ros::NodeHandle* n){
	gps.begin(GPS_BAUD_RATE);
	delay(100);
	gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
	gps.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
	gps.sendCommand(PMTK_API_SET_FIX_CTL_1HZ);
	gps.sendCommand(PGCMD_ANTENNA);
	
	SensorROS::init(n);
	n->advertise(pubNMEA);
	n->advertise(pubTime);
}

void GPS::updateMeasures(){
	while(serial.available() > 0)
		char r = gps.read();
	
	if (gps.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(gps.lastNMEA());   // this also sets the newNMEAreceived() flag to false
	lastNMEA.data = gps.lastNMEA();
    if (!gps.parse(gps.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
		return;  // we can fail to parse a sentence in which case we should just wait for another
	}
	
	if (timer > millis())  timer = millis();

	// approximately every 2 seconds or so, print out the current stats
	if (millis() - timer > 1000) {
		timer = millis();
		setTime((int)gps.hour, (int)gps.minute, (int)gps.seconds, (int)gps.day, (int)gps.month, (int)gps.year);
		time = now();
		//Serial.print("Fix: "); Serial.print((int)GPS.fix);
		//Serial.print(" quality: "); Serial.println((int)GPS.fixquality); 
		status = (int)gps.fix-1;
		timeU.data = time;
		if (gps.fix) {
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
			GPS_speed = gps.speed*1.852*0.27777777778;
			GPS_track = gps.angle;
			GPS_alt = gps.altitude;
			nbSatellites = (int)gps.satellites;
		}
	}
	
	//}
	//else {
	//  Logger::Message(F("\t"), F("GPS:"), F("Lost!"), 1);
	//}
	//}
}

void GPS::updateTest(){
	GPS_lat = 50.365595;
	GPS_long = -4.143274;
}

void GPS::communicateData(){
	msg.latitude = GPS_lat;
	msg.longitude = GPS_long;
	msg.altitude = GPS_alt;
	
	msg.track = GPS_track;
	msg.speed = GPS_speed;
	msg.time = time;
	msg.hdop = hdop;
	
	msg.status.satellites_used = nbSatellites;
	msg.status.satellites_visible = nbSatellites;
	/////********COULD BUILD SATELLITES INFO******////
	msg.status.status = status;
	msg.status.motion_source = 1;
	msg.status.orientation_source = 1;
	msg.status.position_source = 1;
	
	msg.position_covariance_type = 0;
	
	msg.header.stamp = nh->now();
	pub.publish(&msg);
	pubNMEA.publish(&lastNMEA);
	pubTime.publish(&timeU);
}
