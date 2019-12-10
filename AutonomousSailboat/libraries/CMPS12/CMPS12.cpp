#include <CMPS12.h>
#include <Wire.h>

void CMPS12::init(ros::NodeHandle* n){
	IMU::init(n);
}

void CMPS12::updateMeasure(){
	Wire.beginTransmission(address);
	Wire.write(0x02);
	Wire.endTransmission();
	Wire.requestFrom((uint8_t)address, (uint8_t)22);
	int yaw = Wire.read();
	yaw <<= 8;
	yaw += Wire.read();

	int pitch = (char)Wire.read();
	int roll = (char)Wire.read();

	int magx = Wire.read();
	magx <<= 8;
	magx += Wire.read();

	int magy = Wire.read();
	magy <<= 8;
	magy += Wire.read();

	int magz = Wire.read();
	magz <<= 8;
	magz += Wire.read();


	int accx = Wire.read();
	accx <<= 8;
	accx += Wire.read();

	int accy = Wire.read();
	accy <<= 8;
	accy += Wire.read();

	int accz = Wire.read();
	accz <<= 8;
	accz += Wire.read();


	int rotx = Wire.read();
	rotx <<= 8;
	rotx += Wire.read();

	int roty = Wire.read();
	roty <<= 8;
	roty += Wire.read();

	int rotz = Wire.read();
	rotz <<= 8;
	rotz += Wire.read();

	angles[0] = yaw*0.001745329;
	angles[1] = pitch*0.01745329;
	angles[2] = roll*0.01745329;

	double cy = cos(-angles[0]*0.5);
	double sy = sin(-angles[0]*0.5);
	double cp = cos(-angles[2]*0.5);
	double sp = sin(-angles[2]*0.5);
	double cr = cos(-angles[1]*0.5);
	double sr = sin(-angles[1]*0.5);

	quat[0] = cy*cp*cr+sy*sp*sr;
	quat[1] = cy*cp*sr-sy*sp*cr;
	quat[2] = sy*cp*sr+cy*sp*cr;
	quat[3] = sy*cp*cr-cy*sp*sr;

	mag[0]= magx;
	mag[1]= magy;
	mag[2]= magz;

	accel[0]= -accx*0.01;
	accel[1]= -accy*0.01;
	accel[2]= accz*0.01;

	rot[0]= rotx;
	rot[1]= roty;
	rot[2]= rotz;

	heading = angles[0];
}

void CMPS12::storeCalibration(){
	uint8_t data[2];
	data[0] = 0x00;
	data[1] = 0xF0;
	Wire.beginTransmission(address);
	Wire.write((char*)data,2);
	Wire.endTransmission();
	delay(20);
	data[1] = 0xF5;
	Wire.beginTransmission(address);
	Wire.write((char*)data,2);
	Wire.endTransmission();
	delay(20);
	data[1] = 0xF6;
	Wire.beginTransmission(address);
	Wire.write((char*)data,2);
	Wire.endTransmission();
	delay(20);
}

void CMPS12::updateTest(){
	quat[0] = 0.980;
	quat[1] = 0.001;
	quat[2] = 0.189;
	quat[3] = -0.063;

	rot[0] = 0.0;
	rot[1] = 0.0;
	rot[2] = 0.0;

	accel[0] = 0.0;
	accel[1] = 0.0;
	accel[2] = 0.0;
}
