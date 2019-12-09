#include <WindSensorSubscriber.h>
#include    <stdlib.h>

void WindSensor::init(ros::NodeHandle* n){
	//SensorROS::init(n);
	nh = n;
	n->subscribe(subWind);
	//n->advertise(pub);
}

void WindSensor::updateAnemometer(){

}

void WindSensor::updateMeasures(){

}

void WindSensor::updateTest(){

}

void WindSensor::communicateData(){
	    msgTest.x = 0;
	    msgTest.y = 0;
	#ifdef WIND_ANEMOMETER_PIN
		msgTest.x = windSpeed*cos(angle);
		msgTest.y = windSpeed*sin(angle);
	#endif
		msgTest.theta = angle;

		pub.publish(&msgTest);
}

void WindSensor::wind_callback(const geometry_msgs::Pose2D& msg){
	angle = msg.theta;
	windSpeed = sqrt(msg.x*msg.x+msg.y*msg.y);
}
