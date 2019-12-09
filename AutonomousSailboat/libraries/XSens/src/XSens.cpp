#include <XSens.h>
#include <Wire.h>

void XSens::init(ros::NodeHandle* n){
	IMU::init(n);
	//xbus.goToMeas();
	//xbus.startBiasEstimation();
}

void XSens::updateMeasure(){
	xbus.read();
	memcpy(angles,xbus.angles,3*sizeof(float));
	memcpy(quat,xbus.quat,4*sizeof(float));
	memcpy(rot,xbus.rot,3*sizeof(float));
	//memcpy(dv,xbus.dv,3*sizeof(float));
	memcpy(accel,xbus.accel,3*sizeof(float));
	memcpy(mag,xbus.mag,3*sizeof(float));
}

void XSens::startCalibration(){
	xbus.startCalibration();
}

void XSens::stopCalibration(){
	xbus.stopCalibration();
}

void XSens::storeCalibration(){
	xbus.storeCalibration();
}
void XSens::startGyroCalibration(){
	xbus.startBiasEstimation();
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

void XSens::setGPSPosition(float lat, float longitude, float alt){
	xbus.setLatLongAlt(lat,longitude,alt);
}
