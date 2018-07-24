#include <GPS.h>

void GPS::init(ros::NodeHandle* n){
	gps.begin(GPS_BAUD_RATE);
	
	SensorROS::init(n);
}

void GPS::updateMeasures(){
	while (serial.available() > 0){
		char r = gps.read();
	}
	
	
	if (gps.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    if (!gps.parse(gps.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
		updateMeasures();
      //return;  // we can fail to parse a sentence in which case we should just wait for another
	}
    
    GPS_lat = 0;
    GPS_long = 0;
	
	/*
	if (gps.location.isValid()) {
		// Catching location:
		GPS_lat = gps.location.lat();  // In degrees
		GPS_long = gps.location.lng();  // In degrees
		if(GPS_latInit == 0)
			GPS_latInit = GPS_lat;
		if(GPS_longInit == 0)
			GPS_longInit = GPS_long;
		
		// Changing the reference:
		GPS_PosX = (double)(EARTH_RADIUS*(GPS_lat - GPS_latInit)*(DEG_TO_RAD)*cos(GPS_longInit)); // x = EARTH_RADIUS*(a2-a1)*(pi/180)*cos(b1)
		GPS_PosY = (double)(EARTH_RADIUS*(GPS_long - GPS_longInit)*(DEG_TO_RAD)); // y = EARTH_RADIUS*(b2-b1)*pi/180
		
		// Feedback:
		//    Logger::Log(2, F("GPS X:"), String(*p_GPS_PosX));
		//    Logger::Log(2, F("GPS Y:"), String(*p_GPS_PosY));
	}
	if (gps.altitude.isValid()) {
		GPS_alt = gps.altitude.meters();
		if(GPS_altInit == 0)
			GPS_altInit = GPS_alt;
	}
	
	if(gps.location.age() > 1500)
		status = -1;
	else
		status = 0;
	
	if (gps.time.isValid())
		time = gps.time.value();
	
	if (gps.course.isValid())
		GPS_track = gps.course.deg();
	
	if (gps.hdop.isValid())
		hdop = gps.hdop.value();
	
	if (gps.speed.isValid())
		GPS_speed = gps.speed.mps();
	
	if (gps.satellites.isValid())
		nbSatellites = gps.satellites.value();*/
	
	
    time = gps.milliseconds;
    //Serial.print("Fix: "); Serial.print((int)GPS.fix);
    //Serial.print(" quality: "); Serial.println((int)GPS.fixquality); 
    if (gps.fix) {
      GPS_lat = gps.latitudeDegrees;
      GPS_long = gps.longitudeDegrees;
		if(GPS_altInit == 0)
			GPS_altInit = GPS_alt;
		if(GPS_latInit == 0)
			GPS_latInit = GPS_lat;
		if(GPS_longInit == 0)
      GPS_speed = gps.speed;
      GPS_track = gps.angle;
      GPS_alt = gps.altitude;
      nbSatellites = (int)gps.satellites;
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
}
