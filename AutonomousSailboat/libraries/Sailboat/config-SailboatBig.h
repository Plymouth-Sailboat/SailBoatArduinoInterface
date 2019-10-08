#ifndef CONFIG_H
#define CONFIG_H


/**********************************************ID********************************************/
/**SENSORS**/
/***********/
#define NB_SENSORS		4

#define SENSOR_WINDSENSOR	0
#define SENSOR_GPS			1
#define SENSOR_IMU            2
#define SENSOR_BATTERY            3

#define NB_SENSORS_NOT_ROS		1

#define SENSOR_RC			0

/**ACTUATORS**/
/***********/
#define NB_ACTUATORS		3

#define ACTUATOR_RUDDER		0
#define ACTUATOR_SAIL		1
#define ACTUATOR_RUDDER2	2


/**********************************************HARDWARE********************************************/
//RC Transceiver
#define FLYSKY

//Using Grove Shield or not
#define SERVO_SHIELD

//WIND
#define WIND_ANEMOMETER_PIN 3
//RUDDER2
#define RUDDER2_PIN 5

//IMU
//#define CMPS12_IMU
#define XSENS_IMU
//#define USE_ARDUINO_GPS
//#define USE_ARDUINO_WIND
#define FUSE_GPS_IMU


/*****************************************CONFIGURATION****************************************/
/**CONFIG**/
/**********/
//WIND
#define WIND_ANGLE_MIN -180
#define WIND_ANGLE_MAX 180

#define WIND_SENSOR_MIN 0
#define WIND_SENSOR_MAX 1024

//RUDDER
#define RUDDER_POS_MIN  40
#define RUDDER_POS_NEUTRAL  76
#define RUDDER_POS_MAX  120

#define RUDDER_MIN  -30
#define RUDDER_NEUTRAL  0
#define RUDDER_MAX  45

//RUDDER2
#define RUDDER2_POS_MIN  40
#define RUDDER2_POS_NEUTRAL  86
#define RUDDER2_POS_MAX  133

#define RUDDER2_MIN  -10
#define RUDDER2_NEUTRAL  0
#define RUDDER2_MAX  10

//SAIL
#define SAIL_MIN 0
#define SAIL_NEUTRAL SAIL_MIN
#define SAIL_MAX 90

#define WINCH_ANGLE_MIN 74
#define WINCH_ANGLE_NEUTRAL WINCH_ANGLE_MIN
#define WINCH_ANGLE_MAX 138
#define WINCH_DIAMETER 23 // in mm

#endif
