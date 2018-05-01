#include <XSens.h>
#include <Wire.h>

void XSens::init(ros::NodeHandle* n){
	Wire.beginTransmission(address);
	Wire.write(XSENS_PIPE_STATUS);
	Wire.endTransmission();
	
	uint8_t data[4];
	Wire.requestFrom(address,(uint8_t)4);
	while(Wire.available()>0) {
		for(int i = 0; i < 3; i++){
			data[i] = Wire.read();
		}
	}
	if(data[0] == 0x3e)
		wokeUp = true;
	
	SensorROS::init(n);
	n->advertise(pubV);
}

void XSens::updateMeasures(){
	xbus.read();
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
	msg.orientation.x = xbus.quat[0];
	msg.orientation.y = xbus.quat[1];
	msg.orientation.z = xbus.quat[2];
	msg.orientation.w = xbus.quat[3];
	
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
