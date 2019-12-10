#include <IMU.h>
#include <Sailboat.h>
#include <math.h>

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

	float qgq1[3];
	memcpy(accelRaw,accel,3*sizeof(float));
	mulquatvect(accelRaw,quat,qgq1);

	accel[0] = qgq1[0];
	accel[1] =qgq1[1];
	accel[2] = qgq1[2]-9.81f;

	//float meas[4] = {accel[0]*dt + dv[0],accel[1]*dt + dv[1],accel[0],accel[1]};
	float meas[4] = {0,0,accel[0],accel[1]};
	kfP.predict();
	kfP.update(meas);
	dv[0] = meas[0];
	dv[1] = meas[1];
	//dv[0] = kf.updateEstimate(accel[0]*dt+dv[0]);
	//dv[1] = kf1.updateEstimate(accel[1]*dt+dv[1]);
	//dv[2] = kf2.updateEstimate(accel[2]*dt+dv[2]);

	if(abs(dv[0]) > 1000.0 || isnan(dv[0]))
		dv[0] = 0.0;
	if(abs(dv[1]) > 1000.0 || isnan(dv[1]))
		dv[1] = 0.0;
	if(abs(dv[2]) > 1000.0 || isnan(dv[2]))
		dv[2] = 0.0;

	timerDv = millis();
}

void IMU::fuseGPS_IMU(){
	double speed = Sailboat::Instance()->getGPS()->getSpeed();
	double track = -Sailboat::Instance()->getGPS()->getTrack()*M_PI/180.0;
	if(Sailboat::Instance()->getGPS()->getStatus()+1 && !isnan(speed) && abs(speed) < 100.0){
		//dv[0] = kf.updateEstimate(-sin(track)*speed);
		//dv[1] = kf1.updateEstimate(cos(track)*speed);
		//dv[2] = kf2.updateEstimate(0);
		float meas[4] = {-sin(track)*speed,cos(track)*speed,0,0};
		kfP.predict();
		kfP.update(meas);
	}/*else{
		//dv[0] = kf.updateEstimate(0);
		//dv[1] = kf1.updateEstimate(0);
		//dv[2] = kf2.updateEstimate(0);
		//float meas[4] = {0,0,accel[0],accel[1]};
		//kfP.predict();
		//kfP.update(meas);
		//dv[0] = meas[0];
		//dv[1] = meas[1];
	}*/
	if(abs(dv[0]) > 1000.0 || isnan(dv[0]))
		dv[0] = 0.0;
	if(abs(dv[1]) > 1000.0 || isnan(dv[1]))
		dv[1] = 0.0;
	if(abs(dv[2]) > 1000.0 || isnan(dv[2]))
		dv[2] = 0.0;

}


void IMU::communicateData(){
	msg.orientation.w = quat[0];
	msg.orientation.x = quat[1];
	msg.orientation.y = quat[2];
	msg.orientation.z = quat[3];

	msg.angular_velocity.x = rot[0];
	msg.angular_velocity.y = rot[1];
	msg.angular_velocity.z = rot[3];

	msg.linear_acceleration.x = accel[0];
	msg.linear_acceleration.y = accel[1];
	msg.linear_acceleration.z = accel[2];

	velMsg.linear.x = dv[0];
	velMsg.linear.y = dv[1];
	velMsg.linear.z = dv[2];

	velMsg.angular.x = rot[0];
	velMsg.angular.y = rot[1];
	velMsg.angular.z = rot[2];

	msg.header.stamp = nh->now();
	pub.publish(&msg);
	pubV.publish(&velMsg);
}
