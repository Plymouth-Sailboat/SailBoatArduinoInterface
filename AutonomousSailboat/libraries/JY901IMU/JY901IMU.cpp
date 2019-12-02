#include <JY901.h>
#include "JY901IMU.h"
#include <Wire.h>

void JY901IMU::init(ros::NodeHandle* n){
	IMU::init(n);
	JY901.startIIC(address);
	JY901.caliIMU();
}

void JY901IMU::updateMeasure(){
	angles[0] = JY901.getYaw()*0.01745329;
	angles[1] = JY901.getPitch()*0.01745329;
	angles[2] = JY901.getRoll()*0.01745329;

	/*double cy = cos(-angles[0]*0.5);
	double sy = sin(-angles[0]*0.5);
	double cp = cos(-angles[2]*0.5);
	double sp = sin(-angles[2]*0.5);
	double cr = cos(-angles[1]*0.5);
	double sr = sin(-angles[1]*0.5);

	quat[0] = cy*cp*cr+sy*sp*sr;
	quat[1] = cy*cp*sr-sy*sp*cr;
	quat[2] = sy*cp*sr+cy*sp*cr;
	quat[3] = sy*cp*cr-cy*sp*sr;*/

	quat[0] = JY901.getQ0();
	quat[1] = JY901.getQ1();
	quat[2] = JY901.getQ2();
	quat[3] = JY901.getQ3();

	mag[0]= JY901.getMagX();
	mag[1]= JY901.getMagY();
	mag[2]= JY901.getMagZ();

	accel[0]= JY901.getAccX();
	accel[1]= JY901.getAccY();
	accel[2]= JY901.getAccZ();

	rot[0]= JY901.getGyroX();
	rot[1]= JY901.getGyroY();
	rot[2]= JY901.getGyroZ();

	heading = angles[0];
}
void JY901IMU::startCalibration(){
	JY901.caliMag();
}

void JY901IMU::stopCalibration(){
	JY901.quitCali();
}

void JY901IMU::storeCalibration(){
	JY901.saveConf();
}

void JY901IMU::updateTest(){
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
