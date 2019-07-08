#include <IMU.h>

void IMU::mulquatvect(float vec[3], float quat[4], float res[3]){
	float dotuv = quat[1]*vec[0]+quat[2]*vec[1]+quat[3]*vec[2];
	float dotuu = quat[1]*quat[1]+quat[2]*quat[2]+quat[3]*quat[3];
	float crossuv[3] = {quat[2]*vec[2]-quat[3]*vec[1],
	vec[0]*quat[3]-quat[1]*vec[2],
	vec[1]*quat[1]-quat[2]*vec[0]};
	float vcoef = (quat[0]*quat[0]-dotuu);
	res[0] = 2.0f*dotuv*quat[1]+vcoef*vec[0]+2.0f*quat[0]*crossuv[0];
	res[1] = 2.0f*dotuv*quat[2]+vcoef*vec[1]+2.0f*quat[0]*crossuv[1];
	res[2] = 2.0f*dotuv*quat[3]+vcoef*vec[2]+2.0f*quat[0]*crossuv[2];
}

void IMU::measureGravity(){
	if(!wokeUp){
		 timerDv = millis();
		 wokeUp = true;
	}
	double dt = (millis()-timerDv)*0.001;
	float g[4] = {0.0,0.0,0.0,9.81};

	float qgq1[3];
	mulquatvect(accel,quat,qgq1);

	accel[0] = qgq1[0];
	accel[1] = qgq1[1];
	accel[2] = qgq1[2]-9.81f;

	dv[0] = accel[0]*dt+dv[0];
	dv[1] = accel[1]*dt+dv[1];
	dv[2] = accel[2]*dt+dv[2];

	timerDv = millis();
}
