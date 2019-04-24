#include <XSens.h>
#include <Wire.h>

void XSens::init(ros::NodeHandle* n){	
	
	SensorROS::init(n);
	n->advertise(pubV);
}

void XSens::updateMeasures(){
	xbus.read();
	
	if(!wokeUp)
		wokeUp = xbus.wokeUp;
}

void XSens::updateTest(){
	xbus.quat[0] = 0.980;
	xbus.quat[1] = 0.001;
	xbus.quat[2] = 0.189;
	xbus.quat[3] = -0.063;
	
	xbus.rot[0] = 0.0;
	xbus.rot[1] = 0.0;
	xbus.rot[2] = 0.0;
	
	xbus.accel[0] = 0.0;
	xbus.accel[1] = 0.0;
	xbus.accel[2] = 0.0;
}

void XSens::communicateData(){
	msg.orientation.w = xbus.quat[0];
	msg.orientation.x = xbus.quat[1];
	msg.orientation.y = xbus.quat[2];
	msg.orientation.z = xbus.quat[3];
	
	msg.angular_velocity.x = xbus.rot[0];
	msg.angular_velocity.y = xbus.rot[1];
	msg.angular_velocity.z = xbus.rot[3];
	
	msg.linear_acceleration.x = xbus.accel[0];
	msg.linear_acceleration.y = xbus.accel[1];
	msg.linear_acceleration.z = xbus.accel[2];
	
	velMsg.linear.x = xbus.dv[0];
	velMsg.linear.y = xbus.dv[1];
	velMsg.linear.z = xbus.dv[2];
	
	msg.header.stamp = nh->now();
	pub.publish(&msg);
	pubV.publish(&velMsg);
}

void XSens::setGPSPosition(float lat, float longitude, float alt){
	xbus.setLatLongAlt(lat,longitude,alt);
}
